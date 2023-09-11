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

network DADCA_ACK_n50_a8000m2_lpnFn1_7
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
        @display("i=misc/sensor,p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor,p=40.68,139.04");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=101.76,172.30");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=121.82,239.23");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=49.15,307.62");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=48.96,387.72");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=57.97,471.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=79.66,536.10");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=138.08,572.52");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=146.01,477.30");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=7.92,548.45");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=79.52,646.61");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=175.99,631.68");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=202.80,499.31");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=205.71,412.30");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=156.70,338.72");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=15.63,199.53");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=55.17,246.65");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=128.54,403.00");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=236.10,326.05");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=368.81,388.26");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=307.47,426.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=327.45,495.41");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=358.92,570.97");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=394.52,632.73");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=300.89,702.00");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=158.57,733.66");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=289.23,569.28");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=458.13,581.63");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=544.22,565.80");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=630.58,577.57");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=674.81,528.24");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=665.41,459.43");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=745.70,468.43");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=752.94,534.84");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=703.42,616.67");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=729.40,679.45");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=780.21,620.03");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=902.21,575.51");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=904.58,482.73");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=918.16,678.00");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=960.56,764.73");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=1023.05,819.22");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=1091.81,811.44");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1176.15,841.14");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1253.50,829.70");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1251.60,916.15");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1171.38,908.86");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=1208.61,976.12");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1128.88,735.30");
    }

}

