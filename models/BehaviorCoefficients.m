classdef BehaviorCoefficients
    % BehaviorCoefficients  Индивидуальные коэффициенты поведения цели (0..1).

    properties
        Aggressiveness (1, 1) double = 0.5
        Randomness (1, 1) double = 0.5
        Maneuverability (1, 1) double = 0.5
        Inertia (1, 1) double = 0.5
        AltitudePreference (1, 1) double = 0.5
    end

    methods
        function obj = BehaviorCoefficients(varargin)
            if nargin == 0
                return;
            end

            if nargin == 1 && isa(varargin{1}, 'BehaviorCoefficients')
                obj = varargin{1};
                return;
            end

            parser = inputParser;
            addParameter(parser, 'Aggressiveness', 0.5, @(x) BehaviorCoefficients.validateCoefficient(x));
            addParameter(parser, 'Randomness', 0.5, @(x) BehaviorCoefficients.validateCoefficient(x));
            addParameter(parser, 'Maneuverability', 0.5, @(x) BehaviorCoefficients.validateCoefficient(x));
            addParameter(parser, 'Inertia', 0.5, @(x) BehaviorCoefficients.validateCoefficient(x));
            addParameter(parser, 'AltitudePreference', 0.5, @(x) BehaviorCoefficients.validateCoefficient(x));
            parse(parser, varargin{:});

            obj.Aggressiveness = parser.Results.Aggressiveness;
            obj.Randomness = parser.Results.Randomness;
            obj.Maneuverability = parser.Results.Maneuverability;
            obj.Inertia = parser.Results.Inertia;
            obj.AltitudePreference = parser.Results.AltitudePreference;
        end
    end

    methods (Static)
        function obj = createDefault()
            obj = BehaviorCoefficients();
        end

        function obj = createRandom()
            obj = BehaviorCoefficients( ...
                'Aggressiveness', rand(), ...
                'Randomness', rand(), ...
                'Maneuverability', rand(), ...
                'Inertia', rand(), ...
                'AltitudePreference', rand());
        end

        function isValid = validateCoefficient(value)
            isValid = isnumeric(value) && isscalar(value) && value >= 0 && value <= 1;
        end
    end
end
