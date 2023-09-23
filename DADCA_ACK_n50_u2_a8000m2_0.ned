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

network DADCA_ACK_n50_u2_a8000m2_0
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
        @display("i=misc/sensor;p=303.25,7801.47");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1063.36,7903.43");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2441.57,7925.89");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=2980.85,7918.41");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=4409.54,7773.65");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5021.05,7836.63");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=662.54,7605.52");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1555.66,7714.31");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3403.96,7725.00");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3818.12,7622.22");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=202.10,7314.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=2128.69,7401.59");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=3038.03,7350.31");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=1457.57,7214.43");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2719.54,7123.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=3432.99,7049.64");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4204.01,7238.57");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=4802.26,7242.96");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=5245.95,7238.92");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=383.13,6832.40");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=891.04,7002.81");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=1638.21,6839.12");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=2052.58,6929.14");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=74.23,6658.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=996.03,6612.79");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2303.75,6664.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=2968.28,6601.56");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4479.10,6719.79");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=4966.32,6627.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=5658.10,6694.17");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=402.20,6443.79");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1476.09,6459.45");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=3332.39,6473.26");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4162.63,6297.34");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4701.84,6354.27");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5490.13,6293.98");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=72.50,6188.29");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=818.45,6223.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=1639.06,6122.02");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=2040.20,6263.47");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=2667.29,6115.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=3028.88,6024.90");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=3623.26,6230.03");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=5148.77,6053.25");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=359.25,5882.50");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=797.96,5850.15");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=3885.82,5873.14");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=4630.69,5968.32");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5788.33,5864.48");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1297.53,5713.83");
    }

}

