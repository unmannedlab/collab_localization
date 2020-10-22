function obj = run_to(obj, t_final, plot_dt)     
                        
    if obj.plot_fig && obj.save_video
        aviobj = VideoWriter(obj.video_title,'Motion JPEG AVI');
        aviobj.FrameRate = floor( 1 / plot_dt );
        open(aviobj);
    end

    if obj.plot_fig
        figure(1)
    end

%     obj.err_out = cell(length(obj.cars),1);
    
    obj.err_ekf     = cell(length(obj.cars),1);
    obj.err_ekf_lmk = cell(length(obj.cars),1);
    obj.err_dcl     = cell(length(obj.cars),1);
    obj.err_dcl_lmk = cell(length(obj.cars),1);
    
    for i = 1:length(obj.cars)
        obj.err_ekf{i}      = zeros(length( 0:obj.cars{1}.dt:t_final),2);
        obj.err_ekf_lmk{i}  = zeros(length( 0:obj.cars{1}.dt:t_final),2);
        obj.err_dcl{i}      = zeros(length( 0:obj.cars{1}.dt:t_final),2);
        obj.err_dcl_lmk{i}  = zeros(length( 0:obj.cars{1}.dt:t_final),2);
    end
        
    t_int = 1;
    for t = 0:obj.cars{1}.dt:t_final
        for i = 1:length(obj.cars)
            obj.cars{i}.step;
            
            obj.err_ekf{i}(t_int,1)     = norm(obj.cars{i}.ekf_x(1:2)'      - [obj.cars{i}.x_pos, obj.cars{i}.y_pos],2);
            obj.err_ekf_lmk{i}(t_int,1) = norm(obj.cars{i}.ekf_lmk_x(1:2)'  - [obj.cars{i}.x_pos, obj.cars{i}.y_pos],2);
            obj.err_dcl{i}(t_int,1)     = norm(obj.cars{i}.dcl_x(1:2)'      - [obj.cars{i}.x_pos, obj.cars{i}.y_pos],2);
            obj.err_dcl_lmk{i}(t_int,1) = norm(obj.cars{i}.dcl_lmk_x(1:2)'  - [obj.cars{i}.x_pos, obj.cars{i}.y_pos],2);
            
            obj.err_ekf{i}(t_int,2)     = obj.cars{i}.ekf_x(3)      - obj.cars{i}.theta;
            obj.err_ekf_lmk{i}(t_int,2) = obj.cars{i}.ekf_lmk_x(3)  - obj.cars{i}.theta;
            obj.err_dcl{i}(t_int,2)     = obj.cars{i}.dcl_x(3)      - obj.cars{i}.theta;
            obj.err_dcl_lmk{i}(t_int,2) = obj.cars{i}.dcl_lmk_x(3)  - obj.cars{i}.theta;
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