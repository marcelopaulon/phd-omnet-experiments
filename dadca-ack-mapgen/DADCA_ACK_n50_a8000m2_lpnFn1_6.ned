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

network DADCA_ACK_n50_a8000m2_lpnFn1_6
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
        @display("i=misc/sensor;p=44.17,180.65");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=85.91,261.08");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=12.25,274.27");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=18.58,119.85");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=38.32,33.15");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=139.45,160.58");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=213.39,137.36");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=189.32,220.99");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=163.78,305.47");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=226.65,284.22");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=346.73,172.13");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=284.30,174.44");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=361.53,232.73");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=430.66,184.57");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=438.66,118.34");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=523.28,253.87");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=431.90,256.10");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=429.76,328.34");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=471.23,408.01");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=535.39,417.40");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=554.07,494.62");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=634.92,496.19");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=718.52,484.27");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=716.95,393.41");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=608.23,356.32");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=522.08,324.16");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=334.20,63.90");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=131.72,3.32");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=274.59,56.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=197.71,7.33");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=227.61,387.55");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=326.88,287.85");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=268.13,237.89");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=369.14,342.29");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=647.03,428.47");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=684.84,315.74");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=560.13,561.77");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=458.73,484.11");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=497.16,595.26");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=420.56,541.47");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=392.24,626.30");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=604.23,280.58");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=613.70,205.73");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=599.83,125.29");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=538.34,688.13");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=271.53,644.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=295.12,580.80");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=294.97,718.94");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=191.34,660.36");
    }

}

