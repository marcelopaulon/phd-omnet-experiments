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

network DADCA_ACK_n20_u2_a8000m2_1
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
        @display("i=misc/sensor;p=140.20,7878.72");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=616.30,7835.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1267.63,7793.71");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=1671.03,7874.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2164.28,7893.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3317.17,7833.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4164.83,7878.37");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4713.92,7980.60");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5689.08,7870.42");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1014.11,7536.61");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2560.98,7661.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3067.39,7563.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3733.25,7612.18");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4656.68,7630.89");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5314.62,7699.71");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=15.78,7304.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=367.23,7361.82");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1420.26,7302.58");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1970.64,7342.66");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4321.91,7464.13");
    }

}

