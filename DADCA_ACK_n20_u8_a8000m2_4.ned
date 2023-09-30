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

network DADCA_ACK_n20_u8_a8000m2_4
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
        @display("i=misc/sensor;p=939.53,7778.39");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1417.73,7889.61");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2539.18,7849.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3323.83,7788.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4641.81,7844.37");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5325.87,7849.40");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2019.09,7674.91");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3888.98,7618.17");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5135.72,7550.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5762.20,7621.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=488.44,7348.17");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1500.82,7492.62");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3565.77,7428.89");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4252.04,7477.40");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=4707.15,7315.21");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1057.28,7131.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2153.15,7111.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2641.19,7129.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=3043.68,7081.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3510.48,7053.46");
    }

}

