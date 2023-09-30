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

network DADCA_ACK_n20_u4_a8000m2_7
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
        @display("i=misc/sensor;p=399.07,7982.88");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1246.81,7822.96");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1897.55,7848.15");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2943.86,7850.45");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3659.74,7846.38");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4020.62,7807.08");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=5078.01,7940.73");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=956.48,7576.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2317.55,7638.13");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3288.77,7608.97");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4543.47,7667.41");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4954.28,7575.84");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=313.43,7347.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1381.73,7450.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1729.49,7280.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2220.00,7301.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3781.18,7469.38");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4286.78,7396.99");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5382.42,7468.81");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=792.92,7148.31");
    }

}

