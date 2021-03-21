import { FaGithub } from "react-icons/fa";
import { FaLinkedin } from "react-icons/fa";
import "./Header.css";

function Header() {
  return (
    <header className="bg-diff">
      <div className=" jh-nav container space-up-down">
        <h1 className="il-block">AI UAV Flight Simulator</h1>
        <ul className="il-block">
          <li className="d-inline-block">
            <a href="https://github.com/jasminium/formationcontrolui">
              <FaGithub />
            </a>
          </li>
          <li className="d-inline-block ms-3">
            <a href="https://www.linkedin.com/in/john-hartley-4b37219b/">
              <FaLinkedin />
            </a>
          </li>
        </ul>
      </div>
    </header>
  );
}

export default Header;
