function check_controlabilidade(A,B,dim)

if dim == rank(ctrb(A,B))
    disp("O novo sistema é controlável");
else
    disp("O sistema não é controlável");
end

end