classdef Correspondence < handle & matlab.mixin.Heterogeneous
  %Correspondence Object with two 3D corresponding features
  %   See also Point2Point, Point2Line, Point2Plane.

%   Technical:
%   This is a *heterogeneous* object!
%   See also handle, matlab.mixin.Heterogeneous.

methods (Abstract)
  d2 = cost(this,T)
  % Compute cost for this correspondence with transformation T
  
  q = quad(this)
  % Return quadratic form for this correspondence (in terms of vec(T))
  
  
  transformed_pionts = transformPoints(this, T);
  signed_d2 = signed_cost(this, T, centroid)
  centroid = computeCentroid(this)
end

end