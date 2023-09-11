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

network DADCA_ACK_n20_a8000m2_lpnFn1_1
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=74.70,169.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=140.12,223.06");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=95.31,296.14");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=19.01,342.94");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=25.17,249.82");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=178.92,274.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=34.37,107.03");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=77.17,39.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=201.23,118.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=168.95,51.50");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=286.80,99.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=292.53,186.73");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=380.23,118.24");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=339.37,33.07");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=256.32,23.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=421.35,199.46");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=500.32,197.42");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=377.50,271.41");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=470.53,297.45");
    }

}

