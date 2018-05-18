helperOC::HJIPDE_extraArgs def_extraArgs(
		bool accel,
		helperOC::DynSys* p) {

	helperOC::HJIPDE_extraArgs extraArgs;

  // Target set and visualization
	extraArgs.visualize = true;

	if (accel) {
		extraArgs.plotData.plotDims = beacls::IntegerVec{ 1, 1, 0, 0};
		extraArgs.plotData.projpt =
		beacls::FloatVec{p->get_x()[2], p->get_x()[3]};
	}
	else {
		extraArgs.plotData.plotDims = beacls::IntegerVec{ 1, 1, 0};
		extraArgs.plotData.projpt = beacls::FloatVec{p->get_x()[2]};
	}

	extraArgs.deleteLastPlot = true;
	extraArgs.fig_filename = "figs/Car_test";

	extraArgs.execParameters.line_length_of_chunk = 1;
	extraArgs.execParameters.calcTTR = false;
	extraArgs.keepLast = false;
	extraArgs.execParameters.useCuda = false;
	extraArgs.execParameters.num_of_gpus = 0;
	extraArgs.execParameters.num_of_threads = 0;
	extraArgs.execParameters.delayedDerivMinMax =
	  levelset::DelayedDerivMinMax_Disable;
	extraArgs.execParameters.enable_user_defined_dynamics_on_gpu = true;

	return extraArgs;
}
