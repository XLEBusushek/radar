classdef Terrain
    % Terrain  Плавная модель высоты поверхности.

    properties
        XLimits (1, 2) double
        YLimits (1, 2) double
        Amplitude double = 10
        Frequency double = 0.002
        PhaseX double = 0
        PhaseY double = 0
    end

    methods
        function obj = Terrain(xLimits, yLimits, randomSeed)
            arguments
                xLimits (1, 2) double
                yLimits (1, 2) double
                randomSeed (1, 1) double = 0
            end

            obj.XLimits = xLimits;
            obj.YLimits = yLimits;

            if randomSeed ~= 0
                stream = RandStream('mt19937ar', 'Seed', randomSeed);
                obj.PhaseX = 2 * pi * stream.rand();
                obj.PhaseY = 2 * pi * stream.rand();
                obj.Frequency = 0.0015 + 0.001 * stream.rand();
            end
        end

        function height = heightAt(obj, x, y)
            wave = sin(obj.Frequency * x + obj.PhaseX) + cos(obj.Frequency * y + obj.PhaseY);
            height = 10 + obj.Amplitude * 0.5 * wave;
            height = min(max(height, 0), 20);
        end

        function height = Height(obj, x, y)
            height = obj.heightAt(x, y);
        end
    end
end
