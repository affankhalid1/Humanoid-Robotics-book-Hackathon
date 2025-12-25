import React from 'react';
import NavbarLayout from '@theme/Navbar/Layout';
import NavbarContent from '@theme/Navbar/Content';
import './Navbar.css';

function Navbar() {
  return (
    <NavbarLayout>
      <NavbarContent />
    </NavbarLayout>
  );
}

export default React.memo(Navbar);