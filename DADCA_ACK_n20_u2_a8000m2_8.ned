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

network DADCA_ACK_n20_u2_a8000m2_8
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
        @display("i=misc/sensor;p=975.34,7880.63");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2727.50,7887.22");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4838.30,7796.94");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5374.52,7926.46");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=196.07,7611.69");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=596.13,7526.07");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1351.64,7620.88");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3172.21,7545.08");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3554.54,7621.37");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3932.42,7740.26");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4284.37,7694.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1441.12,7271.92");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1791.20,7485.91");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2377.80,7483.81");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2701.98,7331.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4874.14,7282.32");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=5363.77,7395.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5857.60,7285.64");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=52.49,7165.48");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1060.54,7070.59");
    }

}

