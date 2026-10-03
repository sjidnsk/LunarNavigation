"""Headless PNG renderer; CSV remains the complete UTF-8 data source."""
import csv
import io
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def render(kind,path):
    with open(path,encoding='utf-8',newline='') as stream:rows=list(csv.DictReader(stream))
    figure,axes=plt.subplots(3,3,figsize=(14,9),constrained_layout=True)
    if kind=='learning':
        groups=[('critic1_loss','critic2_loss'),('actor_loss',),('target_mean','fixed_q1_mean','fixed_q2_mean'),
            ('alpha_position','alpha_heading'),('position_entropy','position_entropy_target'),
            ('heading_conditional_entropy','heading_entropy_target'),('position_entropy_gap','heading_entropy_gap'),
            ('position_alpha_at_cap','heading_alpha_at_cap'),('position_max_probability',)]
        for ax,keys in zip(axes.flat,groups):
            for key in keys:
                selected=[r for r in rows if r['metric']==key]
                ax.plot([float(r['last_update']) for r in selected],[float(r['mean']) for r in selected],label=key,marker='o')
            ax.set_title(' / '.join(keys),fontsize=9);ax.set_xlabel('updates');ax.legend(fontsize=7)
    else:
        if kind=='fixed':
            fixed={}
            for row in rows:fixed.setdefault((row['updates'],row['probe_hash']),[]).append(row)
            summaries=[]
            for (updates,probe),actions in fixed.items():
                actor_top=min(actions,key=lambda r:int(r['actor_rank']))
                q_top=min(actions,key=lambda r:int(r['q_rank']))
                summaries.append(dict(updates=updates,q1=sum(float(r['q1']) for r in actions)/len(actions),
                    q2=sum(float(r['q2']) for r in actions)/len(actions),min_q=float(actor_top['min_q']),
                    inside_count=sum(float(r['probability'])*float(r['inside_count']) for r in actions),
                    outside_count=sum(float(r['probability'])*float(r['outside_count']) for r in actions),
                    history=sum(float(r['probability'])*float(r['history']) for r in actions),
                    outside_probability=sum(float(r['probability']) for r in actions if r['inside']=='False'),
                    top_q_actor_rank=int(q_top['actor_rank']),top_actor_q_rank=int(actor_top['q_rank'])))
            rows=summaries
        keys={'fixed':['q1','q2','min_q','inside_count','outside_count','history','outside_probability','top_q_actor_rank','top_actor_q_rank'],
              'evaluation':['final_coverage','completed','zero_gain_ratio','max_zero_gain_run','zero_gain_two_point_loop_steps','outside_steps','max_outside_run','task_exits','task_returns'],
              'reward':['exploration','motion','step','completion','reward_sum','new_area_m2','removed_frontiers_in','removed_frontiers_out','reward_mismatch_count']}[kind]
        xkey='stage' if kind=='evaluation' else 'updates'
        for ax,key in zip(axes.flat,keys):
            groups={}
            for row in rows:
                if row.get(key) in (None,''):continue
                value=float(row[key]=='True') if row[key] in ('True','False') else float(row[key])
                groups.setdefault(float(row[xkey]),[]).append(value)
            xs=sorted(groups);ys=[sum(groups[x])/len(groups[x]) for x in xs]
            ax.plot(xs,ys,marker='o',markersize=3);ax.set_title(key);ax.set_xlabel(xkey)
    for ax in axes.flat:ax.grid(alpha=.2)
    figure.suptitle(kind+' | sampled diagnostics' if kind!='evaluation' else 'Fixed 20 paired cases | complete strict results only')
    if not rows:figure.text(.5,.5,'NO VERIFIED DATA YET',ha='center',fontsize=16)
    output=io.BytesIO();figure.savefig(output,format='png',dpi=110);plt.close(figure)
    return output.getvalue()


if __name__=='__main__':sys.stdout.buffer.write(render(*sys.argv[1:]))
