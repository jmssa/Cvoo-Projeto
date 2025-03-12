function confirmar_valores_finais(r, bb, lambda)

k = 10;

lbb = length(bb);
bb_final_values = bb((lbb-k):lbb);
bb_med = mean(bb_final_values);

llambda = length(lambda);
lambda_final_values = lambda((llambda-k):llambda);
lambda_med = mean(lambda_final_values);

bb_erro = rad2deg(bb_med - r(1));

lambda_erro = rad2deg(lambda_med - r(2));

fprintf("Erro no bb = %fº\n", bb_erro);
fprintf("Erro no lambda = %fº\n", lambda_erro);