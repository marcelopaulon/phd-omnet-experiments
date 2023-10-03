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

network DADCA_ACK_n20_u3_a8000m2_5
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
        @display("i=misc/sensor;p=572.60,7960.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3730.41,7922.97");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4116.54,7788.69");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5058.39,7899.28");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5686.19,7847.20");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=99.90,7558.84");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3544.76,7531.76");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4623.67,7513.21");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5298.51,7508.57");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=507.46,7374.77");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=856.85,7294.72");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1328.11,7346.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1759.76,7342.04");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2194.81,7303.80");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2679.03,7413.58");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3144.43,7302.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4215.82,7352.91");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=5802.90,7297.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5024.38,7201.73");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=220.35,6851.26");
    }

}

