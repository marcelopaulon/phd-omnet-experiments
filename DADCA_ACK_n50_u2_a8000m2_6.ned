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

network DADCA_ACK_n50_u2_a8000m2_6
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
        @display("i=misc/sensor;p=317.98,7903.41");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=920.07,7785.21");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1601.35,7777.87");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3589.74,7854.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4019.27,7891.36");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4556.43,7902.05");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4966.29,7855.38");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5564.85,7956.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=91.20,7627.78");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2113.14,7669.17");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2430.08,7516.42");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3250.98,7662.76");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4278.09,7640.70");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5938.06,7627.87");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3285.50,7295.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3642.48,7335.13");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4033.40,7358.03");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4645.03,7382.75");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5626.91,7374.10");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=670.52,7253.92");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1296.57,7166.59");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2006.34,7164.31");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4441.29,7019.55");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5161.88,7196.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5806.51,7050.76");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2790.62,6966.74");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3478.66,6930.92");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5189.27,6808.72");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=31.13,6671.03");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=977.85,6688.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1459.60,6534.91");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1882.77,6733.56");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3167.18,6723.81");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3749.69,6646.50");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4194.89,6709.09");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=580.00,6470.36");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1128.08,6309.44");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2567.62,6473.44");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2946.98,6370.60");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3620.06,6283.24");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3992.72,6348.43");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4696.73,6477.45");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5623.94,6476.87");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5993.19,6416.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=372.83,6160.65");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1617.00,6186.49");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1974.07,6125.55");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3290.00,6152.12");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5180.99,6227.99");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=50.85,5925.02");
    }

}

