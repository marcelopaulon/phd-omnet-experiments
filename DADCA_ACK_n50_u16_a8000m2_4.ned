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

network DADCA_ACK_n50_u16_a8000m2_4
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
        @display("i=misc/sensor;p=264.34,7960.34");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=760.17,7954.72");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1866.86,7827.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2254.31,7970.35");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=3705.93,7872.53");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=4064.39,7761.02");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=4576.15,7943.25");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=5523.91,7893.42");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=417.85,7611.95");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=1083.59,7704.29");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=2639.08,7658.16");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=4939.11,7691.25");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=108.18,7356.61");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=615.80,7271.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=1410.66,7440.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2034.42,7333.88");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3076.04,7419.65");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3664.12,7294.53");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4320.89,7335.88");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4875.89,7345.07");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=5462.00,7493.07");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=5965.92,7380.68");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=990.52,7104.37");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=2398.73,7228.90");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=175.02,6970.00");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=673.66,6862.95");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2746.83,6844.17");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=3124.45,6848.85");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4266.81,6879.36");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=4652.13,6976.02");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=5073.87,7003.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=5541.51,7000.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=5911.97,6809.59");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=252.28,6522.02");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=1245.02,6614.42");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=2135.20,6730.72");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3654.21,6757.97");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5242.94,6647.82");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=893.57,6296.83");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=1810.60,6498.71");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2411.49,6387.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3018.75,6384.56");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3997.39,6447.16");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=4666.59,6507.10");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5733.48,6314.28");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=1537.41,6144.26");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=2019.55,6062.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2689.35,6089.37");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3498.27,6219.96");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=4118.53,6066.67");
    }

}

