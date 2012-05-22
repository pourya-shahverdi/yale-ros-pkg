#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <fstream>
#include <stdio.h>

IplImage* get_diff( IplImage* old_frame, IplImage* new_frame )
{
  IplImage *diff = cvCreateImage(cvSize(800,600),8,1);
  IplImage *oldGrey = cv
  cvAbsDiff( old_frame, new_frame, diff);
  return diff;
}

int main( int argc, char* argv[] )
{
  ros::init( argc, argv, "eye_player" );
  CvCapture* cap = cvCaptureFromFile("cam2-0624-0654_us.avi");
  std::fstream infile;
  infile.open( argv[1], std::fstream::in );
  char line[1024];
  line[0] = '\0';
  infile.getline( line, 1024 );
  float oldtime = 0;
  int curr_frame = 0;

  IplImage* displayImage = cvCreateImage( cvSize(800, 600), 8, 3);
  IplImage* oldFrame = cvCreateImage( cvSize(800,600), 8, 3 );
  cvNamedWindow( "output", 1 );
  IplImage * frame;
  do
  {
    cvZero( displayImage );
    /*
    CvPoint p1 = cvPoint( 120, 50 );
    CvPoint p2 = cvPoint( 520, 350 );
    cvRectangle( displayImage, p1, p2, CV_RGB( 0,0,256), 1);
    */
    infile.getline( line, 1024 );
    if( strlen(line) > 0 )
    {
      float time, x, y;
      int cue_frame = (int) (time * 29.97 / 1000);
      printf( "cue: %d curr: %d\n", cue_frame, curr_frame );
      while( cue_frame > curr_frame ) {
        if( frame != NULL )
          cvCopy( frame, oldFrame );
        cvGrabFrame(cap);
        curr_frame++;
      }
      frame = cvRetrieveFrame(cap);

      int valid;
      char aoi[64];
      sscanf( line, "%f,%f,%f,%d,%s,", &time, &x, &y, &valid, aoi );
      CvPoint p1 = cvPoint( (int) x-240 -3, (int) y-100 - 3 );
      CvPoint p2 = cvPoint( (int) x-240 +3, (int) y-100 + 3 );
      if( frame != NULL )
      {
        if( valid == 1 )
          cvRectangle( frame, p1, p2, CV_RGB( 0,256,0), 5);
        else
          cvRectangle( frame, p1, p2, CV_RGB( 256,0,0), 1);
        if( oldFrame != NULL )
        {
          IplImage* diff = get_diff(oldFrame, frame);
          cvShowImage( "output", diff );
          cvReleaseImage(&diff);
        }

        int c = cvWaitKey(10);

//        cvReleaseImage(&frame);
      }
      //printf( "line (%d): [%s] [%f/%f/%f/%d/%s]\n", strlen(line), line, time, x, y, valid, aoi );

    }


  }
  while( !infile.eof() );
  infile.close();
  

  return 0;
}
