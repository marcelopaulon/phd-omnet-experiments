package projeto;

import inet.common.geometry.common.SimpleGeographicCoordinateSystem;
import inet.environment.common.PhysicalEnvironment;
import inet.environment.ground.FlatGround;
import inet.node.inet.AdhocHost;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.mobility.contract.IMobility;
import inet.mobility.static.StationaryMobility;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.node.ethernet.Eth100M;
import inet.node.inet.SensorNode;
import inet.node.inet.StandardHost;
import inet.node.inet.WirelessHost;
import inet.physicallayer.apskradio.packetlevel.ApskScalarRadioMedium;
import inet.physicallayer.ieee802154.bitlevel.Ieee802154UwbIrRadioMedium;
import inet.physicallayer.ieee802154.packetlevel.Ieee802154NarrowbandScalarRadioMedium;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.visualizer.integrated.IntegratedMultiVisualizer;
import inet.visualizer.integrated.IntegratedVisualizer;
import inet.physicallayer.contract.packetlevel.IRadioMedium;

network DADCA_ACK_n20_u2_a8000m2_4
{
    parameters:
		int numUAVs = default(1);
    submodules:
        visualizer: IntegratedVisualizer {
            parameters:
                @display("p=100,50");
        }
        coordinateSystem: SimpleGeographicCoordinateSystem {
            parameters:
                @display("p=100,150");
        }
        
        environment: PhysicalEnvironment {
        }
        
        configurator: Ipv4NetworkConfigurator {
            @display("p=0,200");
        }
        radioMedium: <default("Ieee802154NarrowbandScalarRadioMedium")> like IRadioMedium {
            @display("p=200,0");
        }
        quads[numUAVs]: MobileNode {
			
        } 
        
        groundStation: GroundStation {
            @display("i=misc/sensor3");
        }
       
        
    sensors_1: MobileSensorNode {
        @display("i=misc/sensor;p=1469.87,7838.99");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3388.20,7921.06");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4582.70,7837.76");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4973.30,7901.52");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5745.68,7784.88");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=97.83,7522.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=661.39,7686.65");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2096.36,7672.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3068.36,7693.67");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3724.09,7637.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4260.85,7678.44");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1164.92,7329.44");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1647.43,7468.54");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2466.30,7428.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3169.69,7350.92");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4837.89,7401.16");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5318.27,7459.59");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5843.37,7346.11");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=534.40,7244.64");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1948.47,7246.20");
    }

}

