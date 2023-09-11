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

network DADCA_ACK_n50_a8000m2_lpnFn1_9
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
        @display("i=misc/sensor,p=171.31,112.59");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=232.24,195.57");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=320.59,174.60");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=367.44,131.87");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=364.75,64.89");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=464.42,86.15");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=512.04,29.05");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=593.61,68.79");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=575.51,165.87");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=651.88,162.87");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=703.41,101.05");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=683.90,229.76");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=762.81,219.91");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=761.51,286.26");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=796.99,169.79");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=804.41,94.88");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=726.84,347.48");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=745.72,404.55");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=697.16,292.18");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=762.83,42.80");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=863.38,225.29");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=726.19,158.09");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=851.91,53.83");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=909.07,22.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=605.68,265.88");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=529.42,267.60");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=804.14,334.98");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=633.39,356.49");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=685.09,16.50");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=490.97,164.73");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=845.72,290.49");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=844.21,432.48");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=958.05,372.44");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=868.97,354.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=909.46,427.99");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=940.95,539.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=975.97,471.19");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=936.13,314.43");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=1006.34,322.22");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=1034.63,259.50");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=981.72,174.68");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=895.57,156.75");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=935.04,87.22");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=1042.74,45.11");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=1100.58,5.55");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=1072.27,100.17");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=1146.60,91.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=995.45,100.74");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=1217.84,89.95");
    }

}

