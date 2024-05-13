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

network DADCA_ACK_n50_u3_a8000m2_0
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
        @display("i=misc/sensor;p=1044.86,7853.45");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1595.52,7873.25");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3265.69,7931.19");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4389.52,7964.53");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5688.09,7846.79");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1498.69,7519.14");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=1915.76,7604.10");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2467.29,7541.52");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2870.65,7729.98");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3741.88,7607.06");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=4428.71,7558.66");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5134.15,7686.78");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=238.09,7454.35");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2740.16,7278.01");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=3199.95,7425.04");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5524.99,7385.53");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=718.34,7163.40");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=1129.93,7239.93");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1898.60,7243.11");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=2314.93,7053.25");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3324.09,7081.47");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3803.23,7119.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=4215.65,7173.88");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4998.73,7223.85");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=181.29,6927.02");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=1032.79,6789.25");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=1594.23,6809.75");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2965.92,6952.50");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4267.07,6779.44");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4720.08,6863.65");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5707.89,6973.54");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=432.34,6666.22");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2847.94,6619.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3319.97,6536.69");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=3713.31,6582.79");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5359.58,6734.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=1416.51,6426.27");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=2000.13,6463.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4822.09,6384.74");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=390.65,6233.38");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=855.51,6188.56");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1548.62,6041.76");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2608.98,6172.45");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3085.63,6223.61");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3744.26,6172.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4151.97,6259.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4512.53,6032.67");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5354.82,6240.10");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5846.93,6178.69");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=470.42,5885.45");
    }

}

