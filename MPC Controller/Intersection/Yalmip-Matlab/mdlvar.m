classdef mdlvar < handle
	properties (SetAccess = private)
		variable
		const
		type
	end

	methods
		function obj = mdlvar(n,const,type)
			if nargin<3
				type = 'variable';
				if nargin<2
					const = 1;
				end
			end
			obj.variable = sdpvar(1,n);
			obj.const = const;
			obj.type = type;
		end
		
		function out = physical(obj,subs)
			if nargin<2
				subs = 1:numel(obj.variable);
			end
			
			out = obj.variable(subs)*obj.const;
		end
		
		function out = value(obj)
			out = double(obj.variable)*obj.const;
		end
		
		function plot(obj)
			plot(value(obj))
        end
	end
end