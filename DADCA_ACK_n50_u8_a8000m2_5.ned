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

network DADCA_ACK_n50_u8_a8000m2_5
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
        @display("i=misc/sensor;p=1032.25,7951.38");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=1778.35,7782.76");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3956.84,7812.74");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4868.04,7994.76");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=487.20,7520.35");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=846.24,7551.23");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=3094.84,7575.60");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4326.24,7581.75");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=5375.04,7663.65");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=5800.55,7521.01");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=1419.72,7286.97");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3641.16,7459.89");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=4627.91,7279.87");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=5058.93,7302.97");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=364.22,7041.42");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=2101.70,7169.31");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=2626.06,7253.14");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=3176.67,7022.84");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=4153.92,7239.62");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=948.01,6970.23");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=2596.05,6835.92");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=3575.26,6969.40");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=3981.40,6772.43");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=4477.75,6872.58");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=5042.12,6886.51");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=5714.31,6976.09");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=133.93,6680.36");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=507.93,6714.53");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1461.05,6719.06");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=1879.11,6740.82");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2248.06,6668.62");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2927.78,6531.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4638.30,6560.20");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=5534.52,6523.49");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=816.93,6461.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=1931.22,6376.70");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=3330.56,6358.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=4012.64,6412.10");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=5100.36,6487.52");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=394.67,6219.48");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1349.13,6229.98");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=1700.86,6109.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2392.06,6210.62");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2994.81,6023.13");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=4967.85,6022.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=5613.35,6170.60");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=69.52,5826.27");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=836.97,5893.20");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=2200.49,5852.17");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=3535.19,5931.74");
    }

}

