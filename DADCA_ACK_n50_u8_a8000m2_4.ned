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

network DADCA_ACK_n50_u8_a8000m2_4
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
        @display("i=misc/sensor;p=183.26,7826.85");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=525.84,7988.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1111.93,7937.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2314.77,7986.98");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=2911.53,7877.31");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3400.76,7883.65");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=546.28,7630.17");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=945.00,7547.88");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1785.12,7522.61");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2572.22,7687.37");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4565.24,7666.04");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5175.02,7597.77");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=444.84,7273.10");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1331.14,7260.18");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2123.39,7268.62");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2969.07,7452.28");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3467.16,7396.77");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3861.17,7463.49");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5.73,7157.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=918.27,7150.09");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3961.68,7081.13");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4417.10,7220.50");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4836.51,7141.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5492.15,7062.67");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=374.43,6777.18");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1765.98,6872.60");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2253.94,6832.49");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2912.98,6798.21");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3355.94,6957.47");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=3709.43,6830.63");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4369.41,6825.95");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=920.35,6715.02");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4976.60,6677.66");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5679.79,6551.71");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=120.27,6417.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2522.12,6342.00");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3023.33,6418.38");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4005.59,6424.88");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4676.90,6339.81");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5065.71,6331.30");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=718.53,6219.05");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1331.47,6242.55");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1978.36,6111.57");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3434.92,6109.03");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4070.66,6063.63");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5297.68,6031.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5935.82,6093.11");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=28.35,5814.35");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1516.76,5904.45");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=2839.98,5946.86");
    }

}

