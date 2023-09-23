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

network DADCA_ACK_n50_u16_a8000m2_7
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
        @display("i=misc/sensor;p=253.65,7789.20");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1797.49,7848.26");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=4079.29,7815.29");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4540.03,7981.82");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5949.45,7963.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=1474.24,7544.98");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2395.84,7647.54");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=3315.97,7631.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3697.92,7720.96");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4576.56,7561.72");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5400.66,7532.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=5823.37,7622.32");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=310.64,7415.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=816.12,7463.32");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2058.13,7268.22");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2738.57,7294.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=3416.30,7280.61");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4901.91,7383.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1502.45,7058.89");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3924.94,7146.81");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4287.47,7243.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4706.66,7049.58");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5386.55,7138.22");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=5978.01,7074.49");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=498.76,6991.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=958.54,6895.31");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2400.86,6876.37");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=2768.57,6909.24");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=3217.19,6967.56");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=33.73,6655.90");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1198.33,6637.35");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2054.04,6600.82");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3559.31,6693.02");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4066.68,6650.32");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4973.88,6587.14");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5455.11,6670.07");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=799.51,6367.82");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1578.11,6479.18");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2418.48,6335.00");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2770.98,6485.90");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=3161.22,6508.53");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3667.71,6332.57");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=4375.31,6335.21");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5579.74,6339.45");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=5954.90,6479.53");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=143.86,6215.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=1176.95,6187.07");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=2761.42,6048.98");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=3335.29,6063.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=508.55,5975.26");
    }

}

