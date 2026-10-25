/* Execute the actual owner and original resist strategy, including group STOP. */
static void resistance_regression(void)
{
    for(unsigned scenario=0;scenario<15U;scenario++) {
        fresh_ui();testing_axis_actions=0;current_setpoint_error=0;
        unsigned writes5=target_writes[5],before5=enables[5],injected=0;
        click(ICAN_UI_RESIST);
        for(unsigned elapsed=0;elapsed<15000U;elapsed+=10U) {
            if(!injected && scenario==12U && group_phase==GROUP_RESIST_PREPARE) {
                click(ICAN_UI_STOP);injected=1;
            }
            if(!injected && resistance.phase==BR_ENABLE && (scenario==8U || scenario==14U)) {
                if(scenario==8U) click(ICAN_UI_STOP);
                else positions[5]+=0.01f;
                injected=1;
            }
            if(!injected && resistance.phase==BR_MODE && resistance.written && scenario==9U) {
                parameters[5][0]=5.0f;injected=1;
            }
            if(!injected && resistance.phase==BR_ZERO && resistance.written && scenario==13U) {
                currents[5]=0.2f;injected=1;
            }
            step();
            assert(target_writes[5]==writes5); /* Resistance never sends a position target. */
            if(group_phase==GROUP_RESIST_RUN) break;
            if(group_phase==GROUP_FINISHED) break;
        }
        if(scenario==8U || scenario==9U || scenario==12U || scenario==13U || scenario==14U) {
            finish_stop();assert(injected && enables[5]==before5);
            assert(!currents[5] && guard.result!=0);continue;
        }
        assert(group_phase==GROUP_RESIST_RUN && !currents[5] && enables[5]==before5+1);
        assert(parameters[5][0]==3.0f && parameters[5][1]==1.0f && modes[4]==2 && modes[6]==2);
        for(unsigned i=0;i<60U;i++) {
            feedback_velocities[5]=0.4f;positions[5]+=0.004f;step();
            assert(currents[5]<=0 && currents[5]>=-1.0f && !local_fault);
        }
        assert(currents[5]<-0.75f);
        for(unsigned i=0;i<60U;i++) {
            feedback_velocities[5]=-0.4f;positions[5]-=0.004f;step();
            assert(currents[5]>=0 && currents[5]<=1.0f && !local_fault);
        }
        feedback_velocities[5]=0;step();step();assert(currents[5]==0);
        if(scenario==0U) {
            feedback_velocities[5]=0.9f;
            for(unsigned i=0;i<100U;i++) {step();assert(currents[5]>=-1.0f && currents[5]<=0);}
            assert(fabsf(currents[5]+1.0f)<0.00001f);
        }
        if(scenario==0U || scenario==1U) click(ICAN_UI_STOP);
        else if(scenario==2U) positions[4]+=0.04f;
        else if(scenario==3U) feedback5_fault=1;
        else if(scenario==4U) {freeze5=1;feedback5_tick=clock_ms-1000U;}
        else if(scenario==5U) feedback_velocities[5]=1.1f;
        else if(scenario==6U) positions[5]=resistance.origin+1.8f;
        else if(scenario==7U) {ui_heartbeat_at=clock_ms-1600U;ui_heartbeat=g_ican_curl_ui.heartbeat+1;}
        else if(scenario==10U) current_setpoint_error=1;
        else if(scenario==11U) clock_ms+=120001U;
        finish_stop();current_setpoint_error=0;
        assert(!currents[5] && target_writes[5]==writes5);
        assert(guard.result==(scenario==11U ? 0:scenario<=1U || scenario==7U ? B5_CANCELLED:
            scenario==3U || scenario==4U ? B5_FEEDBACK:scenario==10U ? B5_IO:B5_LIMIT));
        assert(local_fault==(scenario>=2U && scenario!=11U));
    }
    puts("PASS resistance: original damping strategy, zero-before-enable/readback, no position targets, opposite current/reversal/deadband/1A cap, 4/6 hold, STOP/fault/stale/heartbeat/time limit and preparation failures");
}
