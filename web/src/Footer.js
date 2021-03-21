import { FaGithub } from "react-icons/fa";
import { FaLinkedin } from "react-icons/fa";
import { FaPlane } from "react-icons/fa";


function Footer() {
    return (
        <div className="bd-footer footer bg-diff2">
        <h4>More...</h4>
        <ul className="bd-footer-links ps-0 mb-3">
        <li className="d-inline-block"><a href="https://gitfront.io/r/johnmatthewhartley/c81e9e54cb2749d718e54ae6517bca517097e1ae/formationplanning/">Animations</a></li>
        </ul>

        <h4>Social</h4>
        <ul className="bd-footer-links ps-0 mb-3">
        <li className="d-inline-block"><a href="https://github.com/jasminium/formationcontrolui"><FaGithub /></a></li>
        <li className="d-inline-block ms-3"><a href="https://www.linkedin.com/in/john-hartley-4b37219b/"><FaLinkedin /></a></li>
        </ul>
        <p style={{paddingTop: '1em'}} className="mb-0">Designed and built by John Hartley using Python, NumPy, SciPy, Flask, <a href="https://gitfront.io/r/johnmatthewhartley/c81e9e54cb2749d718e54ae6517bca517097e1ae/formationplanning/">formationplanning</a>, React, Plotly, Bootstrap, and AWS Elastic Beanstalk.</p>
    </div>);
}

export default Footer
