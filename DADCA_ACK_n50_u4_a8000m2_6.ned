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

network DADCA_ACK_n50_u4_a8000m2_6
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
        @display("i=misc/sensor;p=1942.10,7915.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2324.15,7836.07");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3328.03,7932.52");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4877.72,7938.36");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5364.65,7845.18");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1581.04,7557.32");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2967.28,7586.46");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3558.55,7528.48");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=4094.67,7609.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5055.94,7626.59");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1286.13,7261.43");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2209.32,7333.43");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=2626.68,7493.85");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4744.08,7298.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5490.95,7493.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=185.80,7170.01");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3457.47,7112.81");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4350.31,7136.09");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5217.70,7122.22");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=5760.50,7219.89");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=582.64,7006.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1871.16,6971.20");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2291.52,6985.51");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2975.02,6978.62");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3997.02,6931.40");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=4803.64,6790.17");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5725.80,6774.55");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=92.39,6518.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=603.10,6549.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1214.20,6744.16");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2008.78,6586.18");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2388.32,6628.59");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3547.74,6605.05");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=989.79,6461.62");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2750.21,6447.40");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3875.56,6298.56");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=4538.08,6428.40");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5217.04,6472.58");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=6.22,6092.26");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=553.24,6180.02");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1138.41,6095.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1886.04,6228.66");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2852.26,6060.09");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3525.33,6212.17");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4219.96,6215.27");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4519.96,6026.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=5513.84,6157.09");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1961.13,5792.69");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2443.73,5854.51");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3226.77,5916.64");
    }

}

