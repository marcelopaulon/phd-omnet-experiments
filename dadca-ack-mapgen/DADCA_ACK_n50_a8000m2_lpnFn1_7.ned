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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=21.68,113.28");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=149.48,213.44");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=217.85,204.50");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=245.64,312.76");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=177.61,330.45");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=85.34,323.84");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=101.76,410.09");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=111.88,474.71");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=80.79,562.56");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=172.14,585.14");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=245.94,533.64");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=177.85,476.26");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=166.66,405.48");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=87.68,678.27");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=30.99,658.20");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=267.86,597.86");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=324.99,675.63");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=351.43,616.49");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=328.24,541.93");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=376.14,487.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=435.61,555.07");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=250.35,669.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=259.53,458.72");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=300.78,369.31");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=364.95,414.27");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=355.67,332.43");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=424.51,350.58");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=410.21,267.99");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=527.20,230.68");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=491.89,314.48");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=446.26,179.57");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=402.28,126.53");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=590.45,208.72");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=536.00,152.41");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=560.84,298.63");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=355.00,198.30");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=316.37,279.31");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=281.24,102.66");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=48.87,380.69");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=461.07,83.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=450.58,20.39");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=533.84,26.91");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=320.47,54.51");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=284.33,169.77");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=511.71,389.81");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=454.98,419.99");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=8.27,466.78");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=58.62,230.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=18.65,298.92");
    }

}

