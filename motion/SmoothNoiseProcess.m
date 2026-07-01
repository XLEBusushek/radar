classdef SmoothNoiseProcess
    % SmoothNoiseProcess  Плавный низкочастотный шум (Ornstein-Uhlenbeck).

    methods (Static)
        function value = update(value, sigma, tau, dt, stream)
            arguments
                value {mustBeNumeric}
                sigma (1, 1) double {mustBeNonnegative}
                tau (1, 1) double {mustBePositive}
                dt (1, 1) double {mustBePositive}
                stream (1, 1) RandStream = RandStream.getGlobalStream()
            end

            if tau <= 0
                error('SmoothNoiseProcess:InvalidTau', 'tau must be positive.');
            end

            if isscalar(value)
                value = SmoothNoiseProcess.updateScalar(value, sigma, tau, dt, stream);
            else
                value = value + (-value / tau) * dt + sigma * sqrt(dt) * stream.randn(size(value));
            end
        end
    end

    methods (Static, Access = private)
        function value = updateScalar(value, sigma, tau, dt, stream)
            value = value + (-value / tau) * dt + sigma * sqrt(dt) * stream.randn();
        end
    end
end
