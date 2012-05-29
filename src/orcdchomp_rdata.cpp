#include <openrave/openrave.h>

#include "orcdchomp_rdata.h"

namespace orcdchomp
{

rdata::rdata() : OpenRAVE::XMLReadable("orcdchomp")
{
   this->spheres = 0;
}

rdata::~rdata()
{
   struct sphere * s;
   while (this->spheres)
   {
      s=this->spheres->next;
      free(this->spheres);
      this->spheres = s;
   }
}


rdata_parser::rdata_parser(boost::shared_ptr<rdata> passed_d, const OpenRAVE::AttributesList& atts)
{
   /* save or construct the rdata object */
   this->d = passed_d;
   if(!this->d) this->d.reset(new rdata());
   /* get ready */
   this->inside_spheres = false;
}

OpenRAVE::XMLReadablePtr rdata_parser::GetReadable()
{
   return this->d;
}

OpenRAVE::BaseXMLReader::ProcessElement rdata_parser::startElement(const std::string& name, const OpenRAVE::AttributesList& atts)
{
   if (name == "spheres")
   {
      if (this->inside_spheres) RAVELOG_ERROR("you can't have <spheres> inside <spheres>!\n");
      this->inside_spheres = true;
      return PE_Support;
   }
   if (name == "sphere")
   {
      struct sphere * s;
      if (!this->inside_spheres) { RAVELOG_ERROR("you can't have <sphere> not inside <spheres>!\n"); return PE_Pass; }
      s = (struct sphere *) malloc(sizeof(struct sphere));
      for(OpenRAVE::AttributesList::const_iterator itatt = atts.begin(); itatt != atts.end(); ++itatt)
      {
         if (itatt->first=="link")
            strcpy(s->linkname, itatt->second.c_str());
         else if (itatt->first=="radius")
            s->radius = strtod(itatt->second.c_str(), 0);
         else if (itatt->first=="pos")
            sscanf(itatt->second.c_str(), "%lf %lf %lf", &s->pos[0], &s->pos[1], &s->pos[2]);
         else
            RAVELOG_ERROR("unknown attribute %s=%s!\n",itatt->first.c_str(),itatt->second.c_str());
      }
      /* insert at head of rdata list */
      s->next = this->d->spheres;
      this->d->spheres = s;
      return PE_Support;
   }
   return PE_Pass;
}

void rdata_parser::characters(const std::string& ch)
{
   return;
}

bool rdata_parser::endElement(const std::string& name)
{
   if (name == "orcdchomp") return true;
   if (name == "spheres")
   {
      if (!this->inside_spheres) RAVELOG_ERROR("you can't have </spheres> without matching <spheres>!\n");
      this->inside_spheres = false;
   }
   else if (name == "sphere")
   {
      if (!this->inside_spheres) RAVELOG_ERROR("you can't have </sphere> not inside <spheres>!\n");
   }
   else
      RAVELOG_ERROR("unknown field %s\n", name.c_str());
   return false;
}

} /* namespace orcdchomp */


/* old stuff: */
#if 0
/* default openrave wam7 */
static struct sphere spheres[] =
{
   /* shoulder spheres */
   {"wam0", -1, {0.22, 0.14, 0.346}, 0.15},
   /* upper arm spheres */
   {"wam2", -1, {0.0, -0.20, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.30, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.40, 0.0}, 0.06},
   {"wam2", -1, {0.0, -0.50, 0.0}, 0.06},
   /* elbow knuckle spheres */
   {"wam3", -1, {0.045, 0.0, 0.55}, 0.06},
   /* forearm spheres */
   {"wam4", -1, {-0.045, -0.2, 0.0}, 0.06},
   {"wam4", -1, {-0.045, -0.1, 0.0}, 0.06},
   {"wam4", -1, {-0.045, -0.3, 0.0}, 0.06},
   /* hand sphere */
   {"wam6", -1, {0.0, -0.06-0.04, 0.0}, 0.06},
   /* finger spheres (inner links) */
   {"Finger0-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   {"Finger1-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   {"Finger2-1", -1, { 0.05, -0.01, 0.0}, 0.04},
   /* finger spheres (tip links) */
   {"Finger0-2", -1, { 0.05, 0.0, 0.0}, 0.04},
   {"Finger1-2", -1, { 0.05, 0.0, 0.0}, 0.04},
   {"Finger2-2", -1, { 0.05, 0.0, 0.0}, 0.04}
};
#endif
