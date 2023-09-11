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
        @display("i=misc/sensor,p=189.01,67.63");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=127.51,178.97");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=64.53,163.55");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=107.40,28.33");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=320.64,118.27");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=360.50,170.36");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=289.71,189.73");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=223.16,227.34");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=146.89,285.93");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=76.17,298.83");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=214.75,331.94");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=105.92,404.66");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=87.97,463.68");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=166.42,505.84");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=65.11,602.42");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=156.10,566.56");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=6.48,526.07");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=97.75,529.58");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=250.70,564.63");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=268.36,490.37");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=278.34,317.55");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=372.93,256.45");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=309.21,419.63");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=216.46,439.36");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=347.27,520.87");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=414.90,534.95");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=334.47,582.40");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=253.67,638.95");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=272.01,722.11");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=319.66,776.26");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=307.56,858.39");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=236.14,877.74");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=143.04,843.13");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=145.59,928.28");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=23.92,957.16");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=63.47,1035.31");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=111.46,1083.51");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=205.52,1057.21");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=172.05,1001.43");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=275.19,1142.42");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=243.70,1227.88");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=219.90,1167.93");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=133.00,1147.91");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=102.58,1201.16");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=99.23,1286.23");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=74.99,1361.94");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=163.77,1340.09");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=211.20,1472.31");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=127.73,1503.77");
    }

}

