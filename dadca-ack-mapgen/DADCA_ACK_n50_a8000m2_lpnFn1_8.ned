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
        @display("i=misc/sensor,p=123.92,13.15");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor,p=22.92,142.70");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor,p=89.94,172.84");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor,p=152.33,147.78");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor,p=203.05,203.04");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor,p=206.16,275.53");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor,p=118.28,264.44");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor,p=57.05,265.17");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor,p=4.60,309.46");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor,p=224.89,143.19");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor,p=126.81,324.50");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor,p=165.66,65.64");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor,p=259.28,59.34");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor,p=11.96,40.72");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor,p=325.21,173.82");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor,p=372.63,130.75");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor,p=434.50,166.71");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor,p=503.80,127.31");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor,p=460.47,65.66");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor,p=554.49,86.45");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor,p=562.75,167.70");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor,p=494.26,230.02");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor,p=394.32,11.65");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor,p=652.40,104.72");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor,p=544.36,26.73");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor,p=693.89,28.11");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor,p=649.78,185.26");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor,p=467.72,342.05");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor,p=541.48,379.99");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor,p=485.41,479.16");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor,p=557.53,482.76");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor,p=392.81,451.31");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor,p=378.87,230.05");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor,p=404.70,305.58");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor,p=365.24,67.43");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor,p=305.99,98.34");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor,p=49.61,381.00");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor,p=128.36,454.98");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor,p=74.30,489.21");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor,p=21.67,523.76");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor,p=8.91,449.87");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor,p=75.20,563.72");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor,p=89.15,633.81");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor,p=171.91,645.84");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor,p=27.67,708.28");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor,p=37.92,790.61");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor,p=104.44,761.92");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor,p=124.04,850.60");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor,p=110.34,945.71");
    }

}

