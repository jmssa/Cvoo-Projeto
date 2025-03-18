function check_observabilidade(A,C,dim)

rank(obsv(A,C))
if rank(obsv(A,C)) == dim
    disp("O sistema é observável")
else
    disp("O sistema não é observável")
end

end