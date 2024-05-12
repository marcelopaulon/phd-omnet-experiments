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

network DADCA_ACK_n50_u8_a8000m2_0
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
        @display("i=misc/sensor;p=739.61,7977.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1220.11,7756.05");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1597.23,7893.32");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2060.35,7850.97");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2723.99,7816.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3092.73,7873.46");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3964.86,7854.55");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4638.08,7896.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5489.47,7783.54");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5846.87,7802.27");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=171.19,7742.63");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=730.73,7596.04");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2393.73,7587.05");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=3524.76,7660.64");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5040.91,7678.94");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1884.75,7408.22");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2846.29,7336.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4266.28,7413.17");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5336.67,7415.01");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5930.22,7442.80");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=488.31,7221.64");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1120.27,7067.44");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3453.18,7082.46");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3808.18,7066.28");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=4586.97,7255.70");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=56.76,6862.42");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=431.27,6807.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1300.64,6763.12");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1733.74,6800.43");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2162.71,6881.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2604.19,6997.93");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=3165.90,6794.42");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4730.45,6925.56");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5292.75,6955.42");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5666.60,6895.08");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=890.77,6575.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4217.07,6667.08");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5081.48,6664.81");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2592.77,6310.91");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3262.94,6418.58");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3808.62,6486.06");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4388.00,6303.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=652.94,6221.81");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=1333.39,6143.75");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2940.42,6099.52");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=3737.70,6077.33");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4953.59,6099.87");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5645.46,6255.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=360.61,5949.27");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2335.89,6009.61");
    }

}

