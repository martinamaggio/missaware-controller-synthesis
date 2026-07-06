function c = any_miss(r,s)
    c = any_hit(s-r,s);
    %disp(['    >> INFO: WH constraint AnyMiss(',num2str(r),',',num2str(s),') converted to AnyHit(',num2str(s-r),',',num2str(s),').'])
end
