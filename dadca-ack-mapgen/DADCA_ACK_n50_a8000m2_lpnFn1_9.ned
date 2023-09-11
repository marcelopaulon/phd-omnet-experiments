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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=25.51,53.10");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=83.50,6.26");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=126.46,208.09");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=57.60,242.47");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=153.71,330.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=188.20,398.05");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=247.14,438.87");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=260.67,508.90");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=305.37,551.05");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=216.17,563.85");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=217.43,627.17");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=283.64,348.41");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=318.07,287.35");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=364.85,325.78");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=252.37,235.74");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=314.23,208.17");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=192.62,134.92");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=178.63,28.06");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=282.43,132.60");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=279.40,32.54");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=369.67,160.26");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=441.77,145.84");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=460.63,61.93");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=543.34,39.03");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=392.42,34.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=351.40,79.80");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=400.05,216.10");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=488.18,201.51");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=581.28,180.10");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=546.43,118.30");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=63.02,317.66");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=93.56,400.91");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=71.45,491.45");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=138.50,490.45");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=197.70,474.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=36.54,580.58");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=5.88,510.14");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=29.80,439.59");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=13.80,380.79");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=224.10,320.68");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=45.51,131.14");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=604.82,79.14");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=687.85,64.12");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=633.66,25.61");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=660.72,152.50");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=653.71,237.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=568.99,284.06");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=724.04,282.33");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=817.86,272.89");
    }

}

