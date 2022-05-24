function Instructions() {
  return (
    <div className="bg-diff">
      <div className="container space-up-down">
        <h3>Controls</h3>
        <dl className="row">
          <dt className="col-sm-3">Target X, Target Y, Target Z</dt>
          <dd className="col-sm-9">
            x, y, z centre of the yellow target sphere.
          </dd>
          <dt className="col-sm-3">Target Radius</dt>
          <dd className="col-sm-9">Radius of the yellow sphere.</dd>
          <dt className="col-sm-3">Formation X, Formation Y, Formation Z</dt>
          <dd className="col-sm-9">
            The x, y, z position of the formation shown by the coloured dots.
          </dd>
          <dt className="col-sm-3">Formation Scale</dt>
          <dd className="col-sm-9">Controls the size of formation.</dd>
          <dt className="col-sm-3">Generate</dt>
          <dd className="col-sm-9">Click to run a new simulation.</dd>
        </dl>
      </div>
    </div>
  );
}

export default Instructions;
