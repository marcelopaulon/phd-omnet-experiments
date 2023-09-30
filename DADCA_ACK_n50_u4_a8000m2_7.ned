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

network DADCA_ACK_n50_u4_a8000m2_7
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
        @display("i=misc/sensor;p=363.20,7817.44");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=776.22,7772.00");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1403.15,7861.35");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2062.54,7942.17");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3285.25,7880.99");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=3781.30,7855.78");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4283.77,7974.77");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4724.57,7918.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5227.71,7766.46");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=2442.16,7690.23");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2801.19,7577.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4108.65,7617.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4580.75,7546.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=268.76,7305.37");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=672.96,7296.44");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=1093.78,7476.24");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1645.46,7463.80");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3468.23,7454.59");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1548.68,7013.54");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3231.63,7021.35");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3966.26,7123.30");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4308.76,7244.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4741.64,7064.73");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5091.44,7176.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5561.33,7134.38");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=126.02,6925.23");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=716.15,6949.08");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=1154.65,6916.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2752.21,6879.91");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5916.69,6834.67");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=462.46,6668.58");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1485.13,6522.40");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3467.56,6713.13");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3959.66,6706.53");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4848.08,6548.54");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5239.83,6756.81");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=24.75,6455.84");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=928.95,6393.27");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1918.03,6378.05");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2274.54,6364.04");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2870.54,6352.46");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=4451.94,6427.35");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=5594.12,6464.97");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5982.23,6455.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=526.81,6176.41");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1204.76,6137.11");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3635.35,6117.72");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4774.49,6059.64");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5869.45,6052.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=707.79,5807.98");
    }

}

