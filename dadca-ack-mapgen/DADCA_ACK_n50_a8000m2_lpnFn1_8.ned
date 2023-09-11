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

network DADCA_ACK_n50_a8000m2_lpnFn1_8
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
        @display("i=misc/sensor,p=192.36,100.51");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=220.68,177.22");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=122.99,158.74");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=26.42,137.32");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=110.13,220.66");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=13.21,236.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=298.79,74.11");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=360.61,10.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=428.63,29.31");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=292.98,7.88");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=440.37,97.23");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=500.37,112.14");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=234.22,46.22");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=260.29,120.91");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=356.94,94.33");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=477.65,211.83");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=536.49,192.27");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=472.77,273.86");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=561.26,136.65");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=535.00,255.16");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=507.66,323.46");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=568.00,333.28");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=656.48,307.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=642.73,384.08");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=494.87,429.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=422.48,398.68");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=414.58,487.76");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=440.00,581.57");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=525.48,593.22");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=356.81,605.56");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=276.71,595.47");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=292.97,671.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=347.61,720.70");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=273.33,753.99");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=425.20,737.74");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=459.82,688.02");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=288.84,517.57");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=231.25,478.16");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=211.24,412.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=303.70,444.54");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=324.65,361.04");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=254.45,358.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=348.76,228.09");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=313.70,165.71");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=179.08,222.41");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=400.91,310.03");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=146.89,591.03");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=143.05,443.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=68.15,474.53");
    }

}

