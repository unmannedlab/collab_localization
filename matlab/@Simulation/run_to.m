function obj = run_to(obj, t_final, plot_dt)     
                        
    if obj.plot_fig && obj.save_video
        aviobj = VideoWriter(obj.video_title,'Motion JPEG AVI');
        aviobj.FrameRate = floor( 1 / plot_dt );
        open(aviobj);
    end

    if obj.plot_fig
        figure(1)
    end

    obj.err_out = cell(length(obj.cars),1);
    for i = 1:length(obj.cars)
        obj.err_out{i} = zeros(length( 0:obj.cars{1}.dt:t_final),3);
    end
        
    t_int = 1;
%     obj.draw();
    for t = 0:obj.cars{1}.dt:t_final
        for i = 1:length(obj.cars)
            obj.cars{i}.step;
            obj.err_out{i}(t_int,:) = obj.cars{i}.ekf_x(1:3)' - ...
                [obj.cars{i}.x_pos, obj.cars{i}.y_pos, obj.cars{i}.theta];
        end

        if obj.plot_fig && mod(t, plot_dt) == 0
            
            obj.draw();
            
            if obj.save_video
                frame = getframe(gcf);
                writeVideo(aviobj,frame);
            end
        end
        
        t_int = t_int + 1;
    end

    if obj.plot_fig && obj.save_video
        close(aviobj);
    end
end