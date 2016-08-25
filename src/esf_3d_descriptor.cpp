#include <pcl/io/pcd_io.h>
#include <pcl/features/esf.h> 
#include <pcl/visualization/histogram_visualizer.h>
#include <iostream>
#include <fstream>

int
main(int argc, char** argv)
{
	// Cloud for storing the object.
	pcl::PointCloud<pcl::PointXYZ>::Ptr object(new pcl::PointCloud<pcl::PointXYZ>);

	DIR *dir;
	struct dirent *ent;

	cout << "Generating histograms..." << endl;
	if(atoi(argv[2])){
		ofstream myfile;
		myfile.open("outputDescriptors.csv");
		std::string nameFile;
		std::string delimeter = ".";
		std::string numFile;
		
		if ((dir = opendir (argv[1])) != NULL) {
		  /* print all the files and directories within directory */
		  while ((ent = readdir (dir)) != NULL) {
		  	if(strcmp(ent->d_name,"..") != 0 && strcmp(ent->d_name,".") != 0 && strcmp(ent->d_name,"outputDescriptors.csv") && strcmp(ent->d_name,"outputDescriptors.pcd") != 0){
		  		nameFile = std::string(ent->d_name);
		  		numFile = nameFile.substr(0,nameFile.find(delimeter));
		  		myfile << numFile << ",";
				std::string pathFile = std::string(argv[1])+"/"+ent->d_name;
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(pathFile.c_str(), *object) != 0)
				{
					return -1;
				}
				// ESF estimation object.
				pcl::PointCloud<pcl::ESFSignature640>::Ptr localDescriptor(new pcl::PointCloud<pcl::ESFSignature640>);
				pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
				esf.setInputCloud(object);
			 
				esf.compute(*localDescriptor);
				for(unsigned i = 0; i < localDescriptor->size(); i++){
					for(unsigned j = 0; j < (sizeof(localDescriptor->points[i].histogram)/sizeof(*localDescriptor->points[i].histogram)); j++){
						myfile << localDescriptor->points[i].histogram[j] << ",";
					}
				}
		
				//Even file name -> class 1
				//Odd file name  -> class 0
				if(atoi(numFile.c_str())%2 == 0){
					myfile << "1\n";
				}else{
					myfile << "0\n";
				}
			}
		  }	  
		  
		  myfile.close();
		  closedir (dir);	 
		  
		  cout << "Generation finished" << endl; 
  		  cout << "Output in file: ./outputDescriptors.csv" << endl;	
		} else {
		  /* could not open directory */
		  perror ("");
		  return EXIT_FAILURE;
		}		
	}else{
		pcl::PointCloud<pcl::ESFSignature640>::Ptr globalDescriptor(new pcl::PointCloud<pcl::ESFSignature640>);
		if ((dir = opendir (argv[1])) != NULL) {
		  /* print all the files and directories within directory */
		  while ((ent = readdir (dir)) != NULL) {
		  	if(strcmp(ent->d_name,"..") != 0 && strcmp(ent->d_name,".") != 0 && strcmp(ent->d_name,"outputDescriptors.csv") && strcmp(ent->d_name,"outputDescriptors.pcd") != 0){
				std::string pathFile = std::string(argv[1])+"/"+ent->d_name;
				if (pcl::io::loadPCDFile<pcl::PointXYZ>(pathFile.c_str(), *object) != 0)
				{
					return -1;
				}
				// ESF estimation object.
				pcl::PointCloud<pcl::ESFSignature640>::Ptr localDescriptor(new pcl::PointCloud<pcl::ESFSignature640>);
				pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
				esf.setInputCloud(object);
			 
				esf.compute(*localDescriptor);
				*globalDescriptor += *localDescriptor;
			}
		  }	  
		  
		  closedir (dir);	 
		  
		  pcl::io::savePCDFile("outputDescriptors.pcd", *globalDescriptor);
		  
		  cout << "Generation finished" << endl; 
          cout << "Output in file: ./outputDescriptors.pcd" << endl;	
		}	
	}
}
