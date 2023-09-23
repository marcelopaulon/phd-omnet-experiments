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

network DADCA_ACK_n50_u2_a8000m2_1
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
        @display("i=misc/sensor;p=2013.98,7839.40");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2478.97,7948.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2980.82,7966.91");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=3887.52,7780.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4522.10,7888.64");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5975.51,7826.36");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=575.61,7574.75");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1639.33,7686.77");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2459.99,7568.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3556.32,7530.67");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4480.36,7524.86");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5105.64,7674.65");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=5503.23,7685.20");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1239.32,7271.55");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2969.89,7336.98");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4095.16,7441.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=593.64,7179.78");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1641.74,7180.72");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2130.60,7187.24");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3417.34,7143.06");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4747.41,7145.69");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5185.35,7080.63");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5537.04,7115.62");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=199.83,6926.99");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=2532.52,7007.35");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2931.19,6902.98");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3762.26,6864.25");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4288.05,6892.80");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4984.86,6769.75");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5967.15,6959.54");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1313.15,6713.88");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1962.82,6756.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2334.27,6583.10");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3124.88,6528.63");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3518.99,6586.32");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=4600.64,6561.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=5443.92,6578.36");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5900.45,6590.21");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=251.56,6386.48");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=737.97,6480.23");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1423.95,6300.11");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1752.02,6471.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2696.69,6366.10");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4132.58,6462.64");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5107.35,6437.24");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1024.64,6023.03");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2123.97,6149.48");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3638.00,6100.02");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4292.85,6076.76");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4718.87,6159.34");
    }

}

