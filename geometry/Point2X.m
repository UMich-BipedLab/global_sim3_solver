classdef Point2X < Correspondence
  %Point2X Abstract correspondence between 3D point and 3D primitive
  %   See also Correspondence, Point2Point, Point2Line, Point2Plane.
  
  %#ok<*PROPLC>
  
  properties
    point % always a Point feature
    model % a Feature of the types Point, Line or Plane
  end
  
  methods (Sealed)
    function d2 = cost(this,T)
      % d2 = cost(CORRESPONDENCE,TRANSFORMATION)
      % Compute cost function for this correspondence:
      %   dist(T(point),point)
      
      if numel(this) > 1
        % handle array input
        d2 = NaN(size(this));
        for i=1:numel(this)
          d2(i) = cost(this(i),T);
        end
        return
      end
      
      d2 = distSq(this.model,T*this.point);
    end
    
    function signed_d2 = signed_cost(this, T, centroid)
      % d2 = cost(CORRESPONDENCE,TRANSFORMATION)
      % Compute cost function for this correspondence:
      %   dist(T(point),point)
      
      if numel(this) > 1
        % handle array input
        signed_d2 = NaN(size(this));
        for i=1:numel(this)
          signed_d2(i) = signed_cost(this(i), T, centroid);
        end
        return
      end
      
      signed_d2 = signed_distSq(this.model, T*this.point, centroid);
    end
    
    function transformed_points = transformPoints(this, T)

      
      if numel(this) > 1
        % handle array input
        transformed_points = repmat(Point2Point(),1, size(this, 2));
        for i=1:numel(this)
          transformed_points(i) = transformPoints(this(i), T);
        end
        return
      end
      
      transformed_points = Point2Point(T * this.point, this.model);
    end
    
    
    function showCoordinates(this)
        for i = 1: numel(this)
            fprintf("Point(%i): %.2f, %.2f, %.2f\n", ...
            i, this(i).point.x(1), this(i).point.x(2), this(i).point.x(3))
        end
    end
    
    function centroid = computeCentroid(this)
      n = numel(this);
      centroid.model = zeros(3, 1);
      centroid.point = zeros(3, 1);
      
      for i = 1 : n
          if isa(this(i).point, 'Plane') || isa(this(i).point, 'Line')
              project_pt = this(i).model.project(this(i).point);
          elseif isa(this(i).point, 'Point')
              project_pt = this(i).model;  
          else
              assert("Not a type from Plane, Line, Point")
          end
          assert(isa(this(i).point, 'Point'))
          centroid.model = centroid.model + project_pt.x;
          centroid.point = centroid.point + this(i).point.x;
%           centroid.point
%           plot3(gca(20), ...
%               this(i).point.x(1), this(i).point.x(2), this(i).point.x(3), ...
%               'mv')
      end
      centroid.model = Point(centroid.model/n);
      centroid.point = Point(centroid.point/n);
    end
  end
end
