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
        @display("i=misc/sensor;p=100.00,100.00");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=169.08,141.98");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=110.73,8.48");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=18.94,29.65");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=175.89,11.46");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=274.47,153.10");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=161.17,213.64");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=397.04,168.81");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=410.99,229.69");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=332.00,254.55");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=240.98,238.47");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=125.92,279.97");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=162.51,331.36");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=83.08,387.41");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=79.44,481.03");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=108.42,533.75");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=152.51,426.33");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=210.11,66.41");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=253.85,20.36");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=301.04,59.88");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=493.64,140.11");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=571.75,133.57");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=643.51,92.87");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=706.89,109.96");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=780.95,88.56");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=706.54,25.92");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=644.26,226.88");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=715.44,221.70");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=649.65,345.07");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=746.29,405.91");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=785.14,324.07");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=688.52,427.86");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=716.66,501.88");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=827.54,397.00");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=916.23,390.16");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=866.06,312.84");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=946.33,227.22");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=991.12,290.30");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=931.02,321.25");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=725.92,286.60");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=782.45,247.17");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=826.06,170.60");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=921.94,134.29");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=863.72,115.16");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=775.57,20.26");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=839.92,10.80");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=936.49,12.35");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=1007.90,1.29");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=1047.99,58.64");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1112.23,13.86");
    }

}

