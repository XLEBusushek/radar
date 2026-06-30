classdef TransitionMatrixUtils
    % TransitionMatrixUtils  Нормализация и маскирование матриц переходов.

    methods (Static)
        function P = ensureStochastic(P)
            P = max(P, 0);
            n = size(P, 1);

            for row = 1:n
                rowSum = sum(P(row, :));
                if rowSum > 0
                    P(row, :) = P(row, :) / rowSum;
                else
                    P(row, :) = 0;
                    P(row, row) = 1;
                end
            end
        end

        function P = maskInvalidStates(P, validMask)
            invalidMask = ~validMask;

            P(:, invalidMask) = 0;

            for row = find(invalidMask)'
                P(row, :) = 0;
                P(row, row) = 1;
            end

            P = TransitionMatrixUtils.ensureStochastic(P);
        end
    end
end
