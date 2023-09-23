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

network DADCA_ACK_n50_u2_a8000m2_4
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
        @display("i=misc/sensor;p=1176.68,7832.93");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=3336.21,7812.52");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=3749.70,7952.72");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=927.97,7572.69");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=1549.96,7605.39");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=2164.29,7743.77");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=2724.26,7680.04");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=4439.62,7648.61");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=1140.82,7284.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=3228.24,7435.86");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=3596.91,7457.93");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=3993.99,7354.58");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=602.05,7067.86");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=2358.85,7249.05");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=2819.29,7093.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=4397.58,7251.55");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=4821.47,7126.06");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=76.47,6909.81");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=1303.42,6876.84");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=1901.32,6819.27");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=3034.34,6799.22");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=4087.31,6785.84");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=5257.21,6953.50");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=344.78,6536.05");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=731.65,6689.47");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=2238.70,6596.77");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=3423.12,6741.19");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=4739.31,6558.54");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=5188.47,6576.97");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=729.34,6274.00");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=1102.10,6394.59");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=1938.61,6320.93");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=2855.68,6481.21");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=3826.08,6298.80");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=4221.48,6348.56");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=5684.21,6342.21");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=330.81,6156.25");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1005.73,6045.12");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=2426.45,6033.18");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=3327.18,6043.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=4458.10,6064.48");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=5038.16,6245.49");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=1416.97,5884.47");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2051.54,5818.01");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=2828.86,5981.58");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4142.21,5866.85");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=743.81,5726.01");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=3546.80,5608.58");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=4814.80,5631.29");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=5142.30,5771.14");
    }

}

