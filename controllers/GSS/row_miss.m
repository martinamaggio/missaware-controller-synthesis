function c = row_miss(r,~)
    c = any_hit(1,r+1);
    %disp(['    >> INFO: WH constraint RowMiss(',num2str(r),') converted to AnyHit(1,',num2str(r+1),').'])
end
