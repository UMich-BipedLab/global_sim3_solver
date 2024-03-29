classdef Plane < Feature
  %Plane Object with 3D plane data fields (point and normal)
  %   See also Feature.
  
  %#ok<*PROPLC>
  
  properties
    x % 3D point in the plane (center?)
    n % 3D normal (in S2)
  end
  
  methods
    function this = Plane( point,normal )
      % plane = Plane( point,normal )
      %   Constructor from components
      
      if nargin == 0
        % default values: plane at Z=0
        point  = [0 0 0]';
        normal = [0 0 1]';
      end

      this.x = point;
      this.n = normal;
    end

    function point_proj = project(this,point)
      % x_proj = project( PLANE, POINT )
      % Project POINT into PLANE by finding its closest point
      
      assert( isa(point,'Point'), 'Invalid input point' );
      
      % A point x is projected to a plane (p,n) by p + (I-nn')*(x-p)
      % Or the equivalent value: x - n*(n'*(x-p))
      perpDist = dot(this.n,point.x-this.x,1);
      x_proj = point.x - this.n.*(ones(3,1)*perpDist);
      % Sanity check: sample points must belong to the planes: n'*(x-p)=0
      % norm(dot(this.n,x_proj-this.x,1))
      
      point_proj = Point(x_proj);
    end
    
    function d2 = distSq(this,point)
      % d^2 = distSq(PLANE,POINT)
      % Compute squared distance from POINT to PLANE
      
      perpDist = dot(this.n,point.x-this.x,1);
      d2 = perpDist^2;
    end
    
    function d2 = signed_distSq(this, point, centroid)
      % d^2 = distSq(PLANE,POINT)
      % Compute squared distance from POINT to PLANE
      
%       dir = sign(dot(point.x - this.project(this, point), centroid.x - point.x));
      dir = sign(dot(point.x - this.project(point).x, centroid.x - point.x));
      d2 = dir * distSq(this,point);
    end
    
    function h = plot(this,varargin)
      color = 'k';
      hP = plot3(varargin{1}, this.x(1),this.x(2),this.x(3),[color,'x']);
      % Set line (1m for now)
      L = 1;
      lineData = [ this.x, this.x+L*this.v ]';
      c_foo = num2cell(lineData,1);
      hL = line(c_foo{:},'Color',color);
      
      h = [hP;hL]; % concatenate handles
    end
  end
end