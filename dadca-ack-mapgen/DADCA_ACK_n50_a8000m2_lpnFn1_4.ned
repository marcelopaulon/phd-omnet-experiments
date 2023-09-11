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

network DADCA_ACK_n50_a8000m2_lpnFn1_4
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
        @display("i=misc/sensor,p=79.86,6.27");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=9.58,39.88");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=7.59,108.99");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=158.60,83.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=211.44,18.09");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=245.14,88.47");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=319.54,84.90");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=270.48,154.55");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=351.86,184.75");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=387.47,130.28");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=447.77,108.74");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=461.60,199.53");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=288.40,238.66");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=341.22,5.55");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=402.95,56.73");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=472.71,45.04");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=549.43,31.08");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=454.19,269.68");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=538.91,300.59");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=567.52,208.63");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=656.55,189.61");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=716.42,218.93");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=694.45,136.06");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=814.26,299.75");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=749.72,277.82");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=717.22,364.61");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=669.35,302.63");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=618.98,91.67");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=643.34,18.58");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=549.16,123.42");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=610.61,332.41");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=636.77,249.98");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=139.62,19.16");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=302.57,299.01");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=243.88,353.90");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=193.09,406.88");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=160.23,467.05");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=93.42,439.35");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=33.02,394.34");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=33.17,302.29");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=112.96,261.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=85.15,180.36");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=45.17,243.42");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=178.50,177.19");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=206.10,260.74");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=113.93,371.41");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=119.48,512.53");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=55.79,497.50");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=74.73,568.04");
    }

}

