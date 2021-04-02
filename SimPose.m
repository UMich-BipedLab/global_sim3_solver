classdef SimPose < handle & matlab.mixin.Copyable
  %Pose Object with Sim(3) data fields
  %   A point in the manifold Sim(3) is created from the following data:
  %   - translation t, rotation R and scaling s
  %
  %   Technical:
  %   This is a *handle* *copyable* object, that means that the object
  %   created by the constructor is a handle (pointer) to the data.
  %   If you want to get an *independent* copy of the object (clone)
  %   use the method *copy* in the object.
  %   Intuitively, the pose of an object is inherent to this
  %   and should be assigned to a different object, but another object
  %   could have the same value (copy).
  %
  %   See also handle, matlab.mixin.Copyable.
  
  properties
    sim
    H
  end
  properties (Dependent)
    % Observation components
    s,R,t
  end
  
  methods
    function pose = SimPose( varargin )
      % pose = SimPose( t, R, s)
      %   Constructor from components
      % pose = SimPose( H )
      %   Constructor from transformation matrix
      % pose = SimPose( )
      %   Default constructor, returns identity
      % pose = SimPose( pose )
      %   Copy constructor, returns a clone of the input
      
      if nargin == 0
        s = 1;
        t = zeros(3,1);
        R = eye(3);
      elseif nargin == 1
        if isa(varargin{1},'SimPose')
          % Copy constructor
          pose.sim = varargin{1}.sim;
          return
        else
            s = varargin{1};
            t = zeros(3,1);
            R = eye(3);
        end
%         % If not Pose, the argument should be sim
%         sim = varargin{1};
%         assert( all(size(sim)==[3,4]) || all(sim(end,:)==[0 0 0 1]) );
%         R = sim(1:3,1:3);
%         t = sim(1:3,4);
      else
        t = varargin{1};
        R = varargin{2};
        s = varargin{3};
      end
      assert( isrotation(R) );
      assert( s > 1e-5, "s should not be zero" )
      pose.sim(4,4) = 1/s; % Set H with complete size as hom. matrix
      pose.t = t;
      pose.R = R;
      pose.s = s;
      pose.H = Pose(t, R);
    end
       
    % Additional interface properties
    function s = get.s(pose)
      s = 1/pose.sim(4,4);
    end
    function t = get.t(pose)
      t = pose.sim(1:3,4);
    end
    function set.t(pose,t)
      pose.sim(1:3,4) = t;
    end
    function R = get.R(pose)
      R = pose.sim(1:3,1:3);
    end
    function set.R(pose,R)
      pose.sim(1:3,1:3) = R;
    end
    function set.s(pose,s)
      pose.sim(4,4) = 1/s;
    end
    
    function v = vec(this)
      v = this.s * vec(this.sim(1:3,:)); % this is the same as [s*vec(R);t]
    end
    
    % Overloaded operators for usual arithmetic
    function out = mtimes(this,feature)
      switch class(feature)
        case 'Point'
          % x' = R*x+t
          out = Point(this.s*this.R*feature.x+this.t);
        case 'Line'
          % x' = R*x+t
          % v' = R*v
          out = Line(this.s*this.R*feature.x+this.t,this.R*feature.v);
        case 'Plane'
          % x' = R*x+t
          % n' = R*n
          out = Line(this.s*this.R*feature.x+this.t,this.R*feature.n);
        otherwise
          error('Unknown feature class %s',class(feature))
      end
    end
%     function out = mldivide(this,feature)
%       out = inv(this)*feature; %#ok<MINV>
%     end
    function invPose = inv(pose)
      invPose = SimPose( -1/pose.s*pose.R'*pose.t, pose.R', 1/pose.s);
    end
    
%     function new_pose = oplus( pose, delta )
%       % Compose with pre-multiplication of the increment
%       new_t = pose.t + delta(1:3);
%       new_R = SO3.exp(delta(4:6)) * pose.R;
%       % Save new results into an object of the same type as input
%       c = str2func( class(pose) );
%       new_pose = c(new_t,new_R);
%     end
%     
%     function disp(pose)
%       if numel(pose)==1
%         disp(pose.sim)
%       else
%         s = size(pose);
%         fprintf('%dx%d %s array\n',s,class(pose));
%       end
%     end
%     
%     function plot(pose,varargin)
%       
%       % Setup inputs
%       p = inputParser;
%       p.StructExpand = true;
%       p.KeepUnmatched = true;
%       p.addRequired('pose', @(p)isa(p,'Pose'));
%       p.addParameter('size',0.25);
%       p.addParameter('colors',{'r','g','b'});
%       p.addParameter('tag',[],@ischar);
%       % Parse arguments into options
%       p.parse(pose,varargin{:}); opts = p.Results;
%       size = opts.size;
%       
%       for k=1:3
%         v = size * pose.R(:,k);
%         lin = [ pose.t, pose.t+v ]';
%         patchline(lin(:,1),lin(:,2),lin(:,3),...
%           'edgecolor',opts.colors{k},varargin{:});
%       end
%       % Plot tag text
%       if ~isempty(opts.tag)
%         tag_pos = pose.t + 0.5*size*sum(pose.R,2);
%         text(tag_pos(1),tag_pos(2),tag_pos(3),opts.tag);
%       end
%     end
  end
  
  methods (Static)
    function pose = rand( )
      s = genRandomNumber(0,5,1,1);
      R = rnd.rot();
      rng(2);
      t = randn(3,1);
      pose = SimPose(t,R,s);
    end
  end
end
