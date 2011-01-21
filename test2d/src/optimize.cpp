#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <limits>

using namespace std;

double uniformRand()
{
	return double(rand())/RAND_MAX;
}

double gaussianRand(double mean, double sigm)
{
	// Generation using the Polar (Box-Mueller) method.
	// Code inspired by GSL, which is a really great math lib.
	// http://sources.redhat.com/gsl/
	// C++ wrapper available.
	// http://gslwrap.sourceforge.net/
	double r, x, y;

	// Generate random number in unity circle.
	do
	{
		x = uniformRand()*2 - 1;
		y = uniformRand()*2 - 1;
		r = x*x + y*y;
	}
	while (r > 1.0 || r == 0);

	// Box-Muller transform.
	return sigm * y * sqrt (-2.0 * log(r) / r) + mean;
}

struct TrainingEntry
{
	Eigen::Vector3d odom_tr;
	Eigen::Quaterniond odom_rot;
	Eigen::Vector3d icp_tr;
	Eigen::Quaterniond icp_rot;

	TrainingEntry(){}
	TrainingEntry(std::istream& is)
	{
		double t_x, t_y, t_z, q_x, q_y, q_z, q_w;
		is >> t_x;
		is >> t_y;
		is >> t_z;
		odom_tr = Eigen::Vector3d(t_x, t_y, t_z);
		is >> q_x;
		is >> q_y;
		is >> q_z;
		is >> q_w;
		odom_rot = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
		is >> t_x;
		is >> t_y;
		is >> t_z;
		icp_tr = Eigen::Vector3d(t_x, t_y, t_z);
		is >> q_x;
		is >> q_y;
		is >> q_z;
		is >> q_w;
		icp_rot = Eigen::Quaterniond(q_w, q_x, q_y, q_z);
	}

	void dump(ostream& stream) const
	{
		stream << 
			odom_tr.x() << " " << odom_tr.y() << " " << odom_tr.z() << " " <<
			odom_rot.x() << " " << odom_rot.y() << " " << odom_rot.z() << " " << odom_rot.w() << " " <<
			icp_tr.x() << " " << icp_tr.y() << " " << icp_tr.z() << " " <<
			icp_rot.x() << " " << icp_rot.y() << " " << icp_rot.z() << " " << icp_rot.w();
	}
};

struct TrainingSet: public std::vector<TrainingEntry>
{
	void dump()
	{
		for (TrainingSet::const_iterator it(begin()); it != end(); ++it)
		{
			const TrainingEntry& entry(*it);
			entry.dump(cout);
			cout << "\n";
		}
	}
};

TrainingSet trainingSet;

struct Params
{
	Eigen::Vector3d tr;
	Eigen::Quaterniond rot;
	
	Params():tr(0,0,0),rot(1,0,0,0) {}
	
	// add random noise
	const Params& mutate(double amount = 1.0)
	{
		double count = fabs(gaussianRand(0, 1));
		for (int i = 0; i < int(count + 1); i++)
		{
			int toMutate = rand() % 6;
			switch (toMutate)
			{
				case 0: tr.x() += gaussianRand(0, 0.1) * amount; break;
				case 1: tr.y() += gaussianRand(0, 0.1) * amount; break;
				case 2: tr.z() += gaussianRand(0, 0.1) * amount; break;
				case 3: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, amount * M_PI / 8), Eigen::Vector3d::UnitX())); break;
				case 4: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, amount * M_PI / 8), Eigen::Vector3d::UnitY())); break;
				case 5: rot *= Eigen::Quaterniond(Eigen::AngleAxisd(gaussianRand(0, amount * M_PI / 8), Eigen::Vector3d::UnitZ())); break;
				default: break;
			};
		}
		return *this;
	}
	
	void dump(ostream& stream) const
	{
		stream <<
			tr.x() << " " << tr.y() << " " << tr.z() << " " <<
			rot.x() << " " << rot.y() << " " << rot.z() << " " << rot.w();
	}
};

typedef vector<Params> Genome;

double computeError(const Params& p, const TrainingEntry& e)
{
	const Eigen::Transform3d blk = Eigen::Translation3d(p.tr) * p.rot;
	const Eigen::Transform3d blk_i = Eigen::Transform3d(blk.inverse(Eigen::Isometry));
	const Eigen::Transform3d odom = Eigen::Translation3d(e.odom_tr) * e.odom_rot;
	const Eigen::Transform3d pred_icp = blk * odom * blk_i;
	
	const Eigen::Matrix3d pred_icp_rot_m = pred_icp.matrix().corner(Eigen::TopLeft,3,3);
	const Eigen::Quaterniond pred_icp_rot = Eigen::Quaterniond(pred_icp_rot_m);
	
	//cout << "tr pred icp:\n" << pred_icp.translation() << "\nicp:\n" << e.icp_tr << "\n" << endl;
	//cout << "rot pred icp:\n" << pred_icp_rot << "\nicp:\n" << e.icp_rot << "\n" << endl;
	// FIXME: tune coefficient for rot vs trans
	return 
		((e.icp_tr - pred_icp.translation()).squaredNorm()) +
		e.icp_rot.angularDistance(pred_icp_rot)
	;
}

double computeError(const Params& p)
{
	double error = 0;
	for (TrainingSet::const_iterator it(trainingSet.begin()); it != trainingSet.end(); ++it)
	{
		const TrainingEntry& entry(*it);
		error += computeError(p, entry);
	}
	return error;
}

double evolveOneGen(Genome& genome, double annealing = 1.0, bool showBest = false)
{
	typedef multimap<double, Params> EvaluationMap;
	typedef EvaluationMap::iterator EvaluationMapIterator;
	EvaluationMap evalutationMap;

	double totalError = 0;
	double bestError = numeric_limits<double>::max();
	int bestInd = 0;
	for (size_t ind = 0; ind < genome.size(); ind++)
	{
		double error = computeError(genome[ind]);
		if (error < bestError)
		{
			bestError = error;
			bestInd = ind;
		}

		totalError += error;
		evalutationMap.insert(make_pair(error, genome[ind]));

		/*cout << "E " << ind << " : ";
		genome[ind].dump(cout);
		cout << " = " << error << "\n";*/
	}

	if (showBest)
	{
		//cout << "Best of gen: ";
		genome[bestInd].dump(cout);
		cout << endl;
	}

	assert((genome.size() / 4) * 4 == genome.size());

	size_t ind = 0;
	for (EvaluationMapIterator it = evalutationMap.begin(); ind < genome.size() / 4; ++it, ++ind )
	{
		//cout << "S " << it->first << "\n";
		genome[ind * 4] = it->second;
		genome[ind * 4 + 1] = it->second.mutate(annealing);
		genome[ind * 4 + 2] = it->second.mutate(annealing);
		genome[ind * 4 + 3] = it->second.mutate(annealing);
	}

	return bestError;
}

int main(int argc, char** argv)
{
	srand(time(0));
	
	if (argc != 2)
	{
		cerr << "Usage " << argv[0] << " LOG_FILE_NAME" << endl;
		return 1;
	}
	
	ifstream ifs(argv[1]);
	while (ifs.good())
	{
		TrainingEntry e(ifs);
		if (ifs.good())
			trainingSet.push_back(e);
	}
	//trainingSet.dump();
	
	Genome genome(256);
	//Genome genome(4);
	int generationCount = 64;
	for (int i = 0; i < generationCount; i++)
	{
		cout << i << " best has error " << evolveOneGen(genome, 2. * (double)(generationCount - i) / (double)(generationCount)) << endl;
	}
	/*generationCount = 200;
	for (int i = 0; i < generationCount; i++)
	{
		cout << 50 + i << " best has error " << evolve_one_gen(genome, 1.) << endl;
	}*/
	evolveOneGen(genome, 1.0, true);
	
	return 0;
}