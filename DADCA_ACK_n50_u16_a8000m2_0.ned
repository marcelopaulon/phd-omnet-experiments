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

network DADCA_ACK_n50_u16_a8000m2_0
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
        @display("i=misc/sensor;p=381.11,7929.30");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3427.09,7968.57");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4329.30,7959.83");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=5282.07,7996.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5784.74,7928.83");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=760.42,7510.55");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1363.66,7711.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2973.29,7645.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3703.24,7662.51");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4632.82,7668.25");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5608.94,7534.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=230.05,7262.48");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1585.68,7433.74");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2018.23,7455.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2519.97,7467.77");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3148.44,7333.89");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4083.79,7426.47");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4990.35,7475.51");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=576.80,7073.14");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1618.08,7052.85");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2220.40,7139.88");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=2684.83,7042.96");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3560.23,7152.13");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4576.15,7177.92");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5507.63,7037.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=873.67,6864.86");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1272.14,6990.62");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3941.15,6988.13");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5044.96,6870.93");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5990.86,6931.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=204.82,6664.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=672.24,6562.36");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1145.02,6568.28");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1573.50,6672.11");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2195.56,6629.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2885.29,6751.79");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3440.76,6684.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4562.69,6732.17");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5180.47,6535.40");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5570.97,6551.33");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3989.35,6466.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4349.65,6279.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=181.93,6248.89");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=928.40,6196.65");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=1610.85,6100.98");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=2040.67,6051.35");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2625.24,6250.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3074.00,6263.13");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3538.95,6147.57");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4778.08,6094.21");
    }

}

