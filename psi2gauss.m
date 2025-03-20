function mag = psi2gauss(u)

    if rem(u,2*pi) >= -pi  & rem(u,2*pi) <= pi
        mag = -abs(-(16/pi)*rem(u,2*pi)) + 8;
    elseif rem(u,2*pi) < -pi
        mag = -(16/pi)*(rem(u,2*pi)+2*pi) + 8;
    elseif rem(u,2*pi) > pi
        mag = (16/pi)*(rem(u,2*pi)-2*pi) + 8;
    end

end