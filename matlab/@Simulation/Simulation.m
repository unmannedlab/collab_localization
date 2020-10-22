classdef Simulation < handle
    properties
        cars
        save_video = false; 
        video_title
        plot_fig
        kf_switch
        err_ekf
        err_ekf_lmk
        err_dcl
        err_dcl_lmk
        fh
        tri_lmk
        axis_lim
        track_veh
    end   
    
    methods
        function obj = Simulation(varargin)
            p = inputParser;
            addRequired(p,'scene')
            addOptional(p,'plot_fig', true);
            addOptional(p,'kf_switch', [1,1,1,1]);
            parse(p,varargin{:});
            
            obj.plot_fig  = p.Results.plot_fig;
            obj.kf_switch = p.Results.kf_switch;
            
            if obj.plot_fig
                obj.fh = figure(1);
            end
            
            obj.init(p.Results.scene)
        end
        
        function obj = video(obj, title)
            obj.save_video = true;
            obj.video_title = fullfile('videos',[title,'.avi']);
        end
        
    end
    
end