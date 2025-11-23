/**
* This file is part of gnss_comm.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* gnss_comm is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* gnss_comm is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with gnss_comm. If not, see <http://www.gnu.org/licenses/>.
*/

#include "gnss_ros.hpp"
#include "gnss_comm/GnssDDMeasMsg.h"
#include "gnss_comm/GnssObsProcessedMsg.h"
#include "gnss_comm/GnssTDMeasMsg.h"
#include "gnss_constant.hpp"
#include <vector>

namespace gnss_comm
{
    GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr)
    {
        GnssEphemMsg ephem_msg;
        uint32_t week = 0;
        double tow = 0.0;
        ephem_msg.sat = ephem_ptr->sat;
        tow = time2gpst(ephem_ptr->ttr, &week);
        ephem_msg.ttr.week = week;
        ephem_msg.ttr.tow = tow;
        tow = time2gpst(ephem_ptr->toe, &week);
        ephem_msg.toe.week = week;
        ephem_msg.toe.tow = tow;
        tow = time2gpst(ephem_ptr->toc, &week);
        ephem_msg.toc.week = week;
        ephem_msg.toc.tow = tow;
        ephem_msg.toe_tow = ephem_ptr->toe_tow;
        ephem_msg.week = ephem_ptr->week;
        ephem_msg.iode = ephem_ptr->iode;
        ephem_msg.iodc = ephem_ptr->iodc;
        ephem_msg.health = ephem_ptr->health;
        ephem_msg.code = ephem_ptr->code;
        ephem_msg.ura = ephem_ptr->ura;
        ephem_msg.A = ephem_ptr->A;
        ephem_msg.e = ephem_ptr->e;
        ephem_msg.i0 = ephem_ptr->i0;
        ephem_msg.omg = ephem_ptr->omg;
        ephem_msg.OMG0 = ephem_ptr->OMG0;
        ephem_msg.M0 = ephem_ptr->M0;
        ephem_msg.delta_n = ephem_ptr->delta_n;
        ephem_msg.OMG_dot = ephem_ptr->OMG_dot;
        ephem_msg.i_dot = ephem_ptr->i_dot;
        ephem_msg.cuc = ephem_ptr->cuc;
        ephem_msg.cus = ephem_ptr->cus;
        ephem_msg.crc = ephem_ptr->crc;
        ephem_msg.crs = ephem_ptr->crs;
        ephem_msg.cic = ephem_ptr->cic;
        ephem_msg.cis = ephem_ptr->cis;
        ephem_msg.af0 = ephem_ptr->af0;
        ephem_msg.af1 = ephem_ptr->af1;
        ephem_msg.af2 = ephem_ptr->af2;
        ephem_msg.tgd0 = ephem_ptr->tgd[0];
        ephem_msg.tgd1 = ephem_ptr->tgd[1];
        ephem_msg.A_dot = ephem_ptr->A_dot;
        ephem_msg.n_dot = ephem_ptr->n_dot;
        return ephem_msg;
    }

    EphemPtr msg2ephem(const GnssEphemMsgConstPtr &gnss_ephem_msg)
    {
        EphemPtr ephem(new Ephem());
        ephem->sat = gnss_ephem_msg->sat;
        ephem->ttr = gpst2time(gnss_ephem_msg->ttr.week, gnss_ephem_msg->ttr.tow);
        ephem->toe = gpst2time(gnss_ephem_msg->toe.week, gnss_ephem_msg->toe.tow);
        ephem->toc = gpst2time(gnss_ephem_msg->toc.week, gnss_ephem_msg->toc.tow);
        ephem->toe_tow = gnss_ephem_msg->toe_tow;
        ephem->week = gnss_ephem_msg->week;
        ephem->iode = gnss_ephem_msg->iode;
        ephem->iodc = gnss_ephem_msg->iodc;
        ephem->health = gnss_ephem_msg->health;
        ephem->code = gnss_ephem_msg->code;
        ephem->ura = gnss_ephem_msg->ura;
        ephem->A = gnss_ephem_msg->A;
        ephem->e = gnss_ephem_msg->e;
        ephem->i0 = gnss_ephem_msg->i0;
        ephem->omg = gnss_ephem_msg->omg;
        ephem->OMG0 = gnss_ephem_msg->OMG0;
        ephem->M0 = gnss_ephem_msg->M0;
        ephem->delta_n = gnss_ephem_msg->delta_n;
        ephem->OMG_dot = gnss_ephem_msg->OMG_dot;
        ephem->i_dot = gnss_ephem_msg->i_dot;
        ephem->cuc = gnss_ephem_msg->cuc;
        ephem->cus = gnss_ephem_msg->cus;
        ephem->crc = gnss_ephem_msg->crc;
        ephem->crs = gnss_ephem_msg->crs;
        ephem->cic = gnss_ephem_msg->cic;
        ephem->cis = gnss_ephem_msg->cis;
        ephem->af0 = gnss_ephem_msg->af0;
        ephem->af1 = gnss_ephem_msg->af1;
        ephem->af2 = gnss_ephem_msg->af2;
        ephem->tgd[0] = gnss_ephem_msg->tgd0;
        ephem->tgd[1] = gnss_ephem_msg->tgd1;
        ephem->A_dot = gnss_ephem_msg->A_dot;
        ephem->n_dot = gnss_ephem_msg->n_dot;
        return ephem;
    }

    GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr)
    {
        GnssGloEphemMsg glo_ephem_msg;
        uint32_t week = 0;
        double tow = 0.0;
        glo_ephem_msg.sat = glo_ephem_ptr->sat;
        tow = time2gpst(glo_ephem_ptr->ttr, &week);
        glo_ephem_msg.ttr.week = week;
        glo_ephem_msg.ttr.tow = tow;
        tow = time2gpst(glo_ephem_ptr->toe, &week);
        glo_ephem_msg.toe.week = week;
        glo_ephem_msg.toe.tow = tow;
        glo_ephem_msg.freqo = glo_ephem_ptr->freqo;
        glo_ephem_msg.iode = glo_ephem_ptr->iode;
        glo_ephem_msg.health = glo_ephem_ptr->health;
        glo_ephem_msg.age = glo_ephem_ptr->age;
        glo_ephem_msg.ura = glo_ephem_ptr->ura;
        glo_ephem_msg.pos_x = glo_ephem_ptr->pos[0];
        glo_ephem_msg.pos_y = glo_ephem_ptr->pos[1];
        glo_ephem_msg.pos_z = glo_ephem_ptr->pos[2];
        glo_ephem_msg.vel_x = glo_ephem_ptr->vel[0];
        glo_ephem_msg.vel_y = glo_ephem_ptr->vel[1];
        glo_ephem_msg.vel_z = glo_ephem_ptr->vel[2];
        glo_ephem_msg.acc_x = glo_ephem_ptr->acc[0];
        glo_ephem_msg.acc_y = glo_ephem_ptr->acc[1];
        glo_ephem_msg.acc_z = glo_ephem_ptr->acc[2];
        glo_ephem_msg.tau_n = glo_ephem_ptr->tau_n;
        glo_ephem_msg.gamma = glo_ephem_ptr->gamma;
        glo_ephem_msg.delta_tau_n = glo_ephem_ptr->delta_tau_n;
        return glo_ephem_msg;
    }

    GloEphemPtr msg2glo_ephem(const GnssGloEphemMsgConstPtr &gnss_glo_ephem_msg)
    {
        GloEphemPtr glo_ephem(new GloEphem());
        glo_ephem->sat = gnss_glo_ephem_msg->sat;
        glo_ephem->ttr = gpst2time(gnss_glo_ephem_msg->ttr.week, gnss_glo_ephem_msg->ttr.tow);
        glo_ephem->toe = gpst2time(gnss_glo_ephem_msg->toe.week, gnss_glo_ephem_msg->toe.tow);
        glo_ephem->freqo = gnss_glo_ephem_msg->freqo;
        glo_ephem->iode = gnss_glo_ephem_msg->iode;
        glo_ephem->health = gnss_glo_ephem_msg->health;
        glo_ephem->age = gnss_glo_ephem_msg->age;
        glo_ephem->ura = gnss_glo_ephem_msg->ura;
        glo_ephem->pos[0] = gnss_glo_ephem_msg->pos_x;
        glo_ephem->pos[1] = gnss_glo_ephem_msg->pos_y;
        glo_ephem->pos[2] = gnss_glo_ephem_msg->pos_z;
        glo_ephem->vel[0] = gnss_glo_ephem_msg->vel_x;
        glo_ephem->vel[1] = gnss_glo_ephem_msg->vel_y;
        glo_ephem->vel[2] = gnss_glo_ephem_msg->vel_z;
        glo_ephem->acc[0] = gnss_glo_ephem_msg->acc_x;
        glo_ephem->acc[1] = gnss_glo_ephem_msg->acc_y;
        glo_ephem->acc[2] = gnss_glo_ephem_msg->acc_z;
        glo_ephem->tau_n = gnss_glo_ephem_msg->tau_n;
        glo_ephem->gamma = gnss_glo_ephem_msg->gamma;
        glo_ephem->delta_tau_n = gnss_glo_ephem_msg->delta_tau_n;
        return glo_ephem;
    }

    GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas)
    {
        GnssMeasMsg gnss_meas_msg;
        for (ObsPtr obs : meas)
        {
            GnssObsMsg obs_msg;
            uint32_t week = 0;
            double tow = time2gpst(obs->time, &week);
            obs_msg.time.week = week;
            obs_msg.time.tow = tow;
            obs_msg.sat     = obs->sat;
            obs_msg.freqs   = obs->freqs;
            obs_msg.CN0     = obs->CN0;
            obs_msg.LLI     = obs->LLI;
            obs_msg.code    = obs->code;
            obs_msg.psr     = obs->psr;
            obs_msg.psr_std = obs->psr_std;
            obs_msg.cp      = obs->cp;
            obs_msg.cp_std  = obs->cp_std;
            obs_msg.dopp    = obs->dopp;
            obs_msg.dopp_std = obs->dopp_std;
            obs_msg.status  = obs->status;

            gnss_meas_msg.meas.push_back(obs_msg);
        }
		gnss_meas_msg.header.stamp = ros::Time(time2sec(meas[0]->time));
        return gnss_meas_msg;
    }

    std::vector<ObsPtr> msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg)
    {
        std::vector<ObsPtr> meas;
        for (size_t i = 0; i < gnss_meas_msg->meas.size(); ++i)
        {
            GnssObsMsg obs_msg = gnss_meas_msg->meas[i];
            ObsPtr obs(new Obs());
            obs->time       = gpst2time(obs_msg.time.week, obs_msg.time.tow);
            obs->sat        = obs_msg.sat;
            obs->freqs      = obs_msg.freqs;
            obs->CN0        = obs_msg.CN0;
            obs->LLI        = obs_msg.LLI;
            obs->code       = obs_msg.code;
            obs->psr        = obs_msg.psr;
            obs->psr_std    = obs_msg.psr_std;
            obs->cp         = obs_msg.cp;
            obs->cp_std     = obs_msg.cp_std;
            obs->dopp       = obs_msg.dopp;
            obs->dopp_std   = obs_msg.dopp_std;
            obs->status     = obs_msg.status;
            
            meas.push_back(obs);
        }
        return meas;
    }


    GnssTimePulseInfoMsg tp_info2msg(const TimePulseInfoPtr &tp_info)
    {
        GnssTimePulseInfoMsg tp_info_msg;
        tp_info_msg.time.tow = time2gpst(tp_info->time, &(tp_info_msg.time.week));
        tp_info_msg.utc_based = tp_info->utc_based;
        tp_info_msg.time_sys = tp_info->time_sys;
        return tp_info_msg;
    }

    TimePulseInfoPtr msg2tp_info(const GnssTimePulseInfoMsgConstPtr &gnss_tp_info_msg)
    {
        TimePulseInfoPtr tp_info(new TimePulseInfo());
        tp_info->time = gpst2time(gnss_tp_info_msg->time.week, gnss_tp_info_msg->time.tow);
        tp_info->utc_based = gnss_tp_info_msg->utc_based;
        tp_info->time_sys = gnss_tp_info_msg->time_sys;
        return tp_info;
    }

    GnssBestXYZMsg best_xyz2msg(const BestXYZPtr &best_xyz)
    {
        GnssBestXYZMsg gnss_best_xyz_msg;
        gnss_best_xyz_msg.header.stamp = ros::Time(time2sec(best_xyz->time));
        gnss_best_xyz_msg.header.frame_id = "ECEF";
        for (size_t i = 0; i < 3; ++i)
        {
            gnss_best_xyz_msg.pos.push_back(best_xyz->pos[i]);
            gnss_best_xyz_msg.pos_sigma.push_back(best_xyz->pos_sigma[i]);
            gnss_best_xyz_msg.vel.push_back(best_xyz->vel[i]);
            gnss_best_xyz_msg.vel_sigma.push_back(best_xyz->vel_sigma[i]);
        }
        gnss_best_xyz_msg.num_svs = best_xyz->num_soln_svs;

        return gnss_best_xyz_msg;
    }

    GnssPVTSolnMsg pvt2msg(const PVTSolutionPtr &pvt_soln)
    {
        GnssPVTSolnMsg pvt_msg;
        pvt_msg.time.tow = time2gpst(pvt_soln->time, &(pvt_msg.time.week));
        pvt_msg.fix_type = pvt_soln->fix_type;
        pvt_msg.valid_fix = pvt_soln->valid_fix;
        pvt_msg.diff_soln = pvt_soln->diff_soln;
        pvt_msg.carr_soln = pvt_soln->carr_soln;
        pvt_msg.num_sv = pvt_soln->num_sv;
        pvt_msg.latitude = pvt_soln->lat;
        pvt_msg.longitude = pvt_soln->lon;
        pvt_msg.altitude = pvt_soln->hgt;
        pvt_msg.height_msl = pvt_soln->hgt_msl;
        pvt_msg.h_acc = pvt_soln->h_acc;
        pvt_msg.v_acc = pvt_soln->v_acc;
        pvt_msg.p_dop = pvt_soln->p_dop;
        pvt_msg.vel_n = pvt_soln->vel_n;
        pvt_msg.vel_e = pvt_soln->vel_e;
        pvt_msg.vel_d = pvt_soln->vel_d;
        pvt_msg.vel_acc = pvt_soln->vel_acc;
        return pvt_msg;
    }

    PVTSolutionPtr msg2pvt(const GnssPVTSolnMsgConstPtr &pvt_msg)
    {
        PVTSolutionPtr pvt_soln(new PVTSolution());
        pvt_soln->time = gpst2time(pvt_msg->time.week, pvt_msg->time.tow);
        pvt_soln->fix_type = pvt_msg->fix_type;
        pvt_soln->valid_fix = pvt_msg->valid_fix;
        pvt_soln->diff_soln = pvt_msg->diff_soln;
        pvt_soln->carr_soln = pvt_msg->carr_soln;
        pvt_soln->num_sv = pvt_msg->num_sv;
        pvt_soln->lat = pvt_msg->latitude;
        pvt_soln->lon = pvt_msg->longitude;
        pvt_soln->hgt = pvt_msg->altitude;
        pvt_soln->hgt_msl = pvt_msg->height_msl;
        pvt_soln->h_acc = pvt_msg->h_acc;
        pvt_soln->v_acc = pvt_msg->v_acc;
        pvt_soln->p_dop = pvt_msg->p_dop;
        pvt_soln->vel_n = pvt_msg->vel_n;
        pvt_soln->vel_e = pvt_msg->vel_e;
        pvt_soln->vel_d = pvt_msg->vel_d;
        pvt_soln->vel_acc = pvt_msg->vel_acc;
        return pvt_soln;
    }

    GnssSvsMsg svs2msg(const std::vector<SvInfo> &svs)
    {
        GnssSvsMsg svs_msg;
        if (svs.empty())    return svs_msg;
        svs_msg.header.stamp = ros::Time(time2sec(svs[0].time));
        for (auto & sv : svs)
        {
            svs_msg.sat.push_back(sv.sat);
            svs_msg.freqo.push_back(sv.freqo);
            svs_msg.health.push_back(sv.health);
            svs_msg.elev_degree.push_back(sv.elev_degree);
            svs_msg.az_degree.push_back(sv.az_degree);
            svs_msg.sig_mask.push_back(sv.sig_mask);
        }
        return svs_msg;
    }

	GnssObsProcessedMsg SatelliteData2ObsProcessedMsg(const SatelliteData &sat_data)
	{
		GnssObsProcessedMsg obs_processed_msg;
		uint32_t week = 0;
		double tow = time2gpst(sat_data.ttx, &week);
		obs_processed_msg.time.week = week;
		obs_processed_msg.time.tow = tow;
		obs_processed_msg.sat = sat_data.sat_id;

		obs_processed_msg.psr = sat_data.pseudorange;
		obs_processed_msg.psr_std = sat_data.pr_std;
		obs_processed_msg.cp = sat_data.carrier_phase;
		obs_processed_msg.cp_std = sat_data.cp_std;
		obs_processed_msg.dopp = sat_data.doppler;
		obs_processed_msg.dopp_std = sat_data.dopp_std;

		obs_processed_msg.wavelength = sat_data.wavelength;
		obs_processed_msg.sat_pos = {sat_data.sat_pos[0], sat_data.sat_pos[1], sat_data.sat_pos[2]};
		obs_processed_msg.sat_vel = {sat_data.sat_vel[0], sat_data.sat_vel[1], sat_data.sat_vel[2]};
		obs_processed_msg.sat_clk = sat_data.sat_clk;
		obs_processed_msg.sat_ddt = sat_data.sat_ddt;
		obs_processed_msg.elevation = sat_data.elevation;

		return obs_processed_msg;
	}

	GnssDDMeasMsg ddmeas2msg(const std::map<int, DDMeasurement> &dd_obs)
	{
		GnssDDMeasMsg ddmeas_msg;
		for (const auto &item : dd_obs)
		{
			const DDMeasurement &dd_measurement = item.second;
			GnssDDObsMsg dd_obs_msg;
			GnssObsProcessedMsg UmObs = SatelliteData2ObsProcessedMsg(dd_measurement.u_master_SV);
			GnssObsProcessedMsg UiObs = SatelliteData2ObsProcessedMsg(dd_measurement.u_iSV);
			GnssObsProcessedMsg RmObs = SatelliteData2ObsProcessedMsg(dd_measurement.r_master_SV);
			GnssObsProcessedMsg RiObs = SatelliteData2ObsProcessedMsg(dd_measurement.r_iSV);

			dd_obs_msg.UmObs = UmObs;
			dd_obs_msg.UiObs = UiObs;
			dd_obs_msg.RmObs = RmObs;
			dd_obs_msg.RiObs = RiObs;

			uint32_t week = 0;
			double tow = time2gpst(dd_measurement.ttx, &week);
			dd_obs_msg.time.week = week;
			dd_obs_msg.time.tow = tow;
			dd_obs_msg.var_pr = dd_measurement.var_pr;
			dd_obs_msg.var_cp = dd_measurement.var_cp;
			dd_obs_msg.var_dopp = dd_measurement.var_dopp;
			dd_obs_msg.dd_pseudorange = dd_measurement.dd_pseudorange;
			dd_obs_msg.dd_carrier_phase = dd_measurement.dd_carrier_phase;
			dd_obs_msg.dd_doppler = dd_measurement.dd_doppler;

			ddmeas_msg.DDMeas.push_back(dd_obs_msg);
		}
		ddmeas_msg.header.stamp = ros::Time(time2sec(dd_obs.begin()->second.ttx));
		return ddmeas_msg;
	}

	GnssTDMeasMsg tdmeas2msg(const std::map<int, TimeDiffMSMeasurement> &tdmeas_ptr)
	{
		GnssTDMeasMsg tdmeas_msg;
		for (const auto &item : tdmeas_ptr)
		{
			const TimeDiffMSMeasurement &td_measurement = item.second;
			GnssTDObsMsg td_obs_msg;
			GnssObsProcessedMsg masterObs = SatelliteData2ObsProcessedMsg(td_measurement.master_SV);
			GnssObsProcessedMsg slaveObs = SatelliteData2ObsProcessedMsg(td_measurement.iSV);
			GnssObsProcessedMsg lastMasterObs = SatelliteData2ObsProcessedMsg(td_measurement.last_master_SV);
			GnssObsProcessedMsg lastSlaveObs = SatelliteData2ObsProcessedMsg(td_measurement.last_iSV);

			td_obs_msg.CurMObs = masterObs;
			td_obs_msg.CurIObs = slaveObs;
			td_obs_msg.LastMObs = lastMasterObs;
			td_obs_msg.LastIObs = lastSlaveObs;

			uint32_t week = 0;
			double tow = time2gpst(td_measurement.ttx, &week);
			td_obs_msg.time.week = week;
			td_obs_msg.time.tow = tow;

			td_obs_msg.var = td_measurement.var;
			td_obs_msg.td_carrier = td_measurement.tdms_carrier;

			tdmeas_msg.TDMeas.push_back(td_obs_msg);
		}
		tdmeas_msg.header.stamp = ros::Time(time2sec(tdmeas_ptr.begin()->second.iSV.ttx));
		return tdmeas_msg;
	}
}   // namespace gnss_comm