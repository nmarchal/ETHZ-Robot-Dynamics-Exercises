% proNEu: A tool for rigid multi-body mechanics in robotics.
% 
% Copyright (C) 2017  Marco Hutter, Christian Gehring, C. Dario Bellicoso,
% Vassilios Tsounis
% 
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% any later version.
% 
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
% 
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <http://www.gnu.org/licenses/>.

classdef MoreauTimeSteppingSolver < handle
    
    % Solver interface
    properties (Access = public)
        % Model functions
        system = struct();
        % Excitation functions
        input = struct('int', [], 'ext', []);
        % Solver options
        options = []; 
    end
    
    % Solver internals
    properties (Access = private)
        %
        % TBD
        %
    end
    
    % Solver operations
    methods
        
        function [] = setup(obj, Nx, Nq, funcs, parameters, varargin)
            %
            % TODO
            %
        end
        
        function [data, time, tcomp] = compute(obj, init, tstart, tstop, tstep)
            %
            % TODO
            %
        end
        
    end
    
    
end