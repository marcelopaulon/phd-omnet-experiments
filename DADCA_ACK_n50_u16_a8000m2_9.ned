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

network DADCA_ACK_n50_u16_a8000m2_9
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
        @display("i=misc/sensor;p=39.86,7912.57");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=877.36,7889.84");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=1515.53,7778.45");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2727.89,7984.42");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4144.45,7822.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5960.16,7924.95");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=470.91,7660.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=2461.47,7704.46");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=2963.78,7634.23");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3599.87,7577.15");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=650.52,7332.33");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1111.38,7359.24");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4111.85,7350.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4845.69,7452.82");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5530.87,7471.81");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=5966.26,7285.83");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=1755.05,7251.37");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2452.71,7256.36");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2816.22,7056.42");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=4541.48,7056.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=1150.07,6908.97");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1719.24,6833.88");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2172.84,6841.05");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=3381.15,6893.31");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3917.81,6818.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5285.18,6984.93");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5758.39,6872.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=68.67,6717.60");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=2526.03,6744.25");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2890.63,6569.56");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=4265.59,6713.47");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=688.25,6489.79");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=1041.63,6287.15");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=1660.42,6319.93");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=2129.42,6462.55");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=3540.27,6410.18");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3992.01,6381.18");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4889.35,6510.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5302.29,6373.55");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=5874.38,6442.99");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=108.08,6191.08");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2167.97,6102.28");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2676.68,6071.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=3021.50,6214.24");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4353.06,6126.46");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5039.63,6034.68");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=174.14,5839.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=691.14,5858.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1086.38,5866.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1572.49,5941.98");
    }

}

