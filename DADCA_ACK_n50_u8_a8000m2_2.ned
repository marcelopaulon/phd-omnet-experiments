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

network DADCA_ACK_n50_u8_a8000m2_2
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
        @display("i=misc/sensor;p=1427.91,7824.48");
    }

    sensors_2: MobileSensorNode {
        @display("i=misc/sensor;p=2543.20,7932.87");
    }

    sensors_3: MobileSensorNode {
        @display("i=misc/sensor;p=2927.91,7755.41");
    }

    sensors_4: MobileSensorNode {
        @display("i=misc/sensor;p=4763.61,7863.38");
    }

    sensors_5: MobileSensorNode {
        @display("i=misc/sensor;p=5273.70,7922.70");
    }

    sensors_6: MobileSensorNode {
        @display("i=misc/sensor;p=5650.96,7881.99");
    }

    sensors_7: MobileSensorNode {
        @display("i=misc/sensor;p=357.65,7550.70");
    }

    sensors_8: MobileSensorNode {
        @display("i=misc/sensor;p=1914.78,7681.00");
    }

    sensors_9: MobileSensorNode {
        @display("i=misc/sensor;p=3602.25,7663.75");
    }

    sensors_10: MobileSensorNode {
        @display("i=misc/sensor;p=4167.13,7650.54");
    }

    sensors_11: MobileSensorNode {
        @display("i=misc/sensor;p=5372.47,7563.11");
    }

    sensors_12: MobileSensorNode {
        @display("i=misc/sensor;p=1100.80,7498.57");
    }

    sensors_13: MobileSensorNode {
        @display("i=misc/sensor;p=1461.33,7268.63");
    }

    sensors_14: MobileSensorNode {
        @display("i=misc/sensor;p=4868.06,7395.47");
    }

    sensors_15: MobileSensorNode {
        @display("i=misc/sensor;p=5848.12,7274.17");
    }

    sensors_16: MobileSensorNode {
        @display("i=misc/sensor;p=145.97,7066.35");
    }

    sensors_17: MobileSensorNode {
        @display("i=misc/sensor;p=579.74,7129.92");
    }

    sensors_18: MobileSensorNode {
        @display("i=misc/sensor;p=2221.77,7140.68");
    }

    sensors_19: MobileSensorNode {
        @display("i=misc/sensor;p=2716.45,7195.63");
    }

    sensors_20: MobileSensorNode {
        @display("i=misc/sensor;p=3200.38,7067.44");
    }

    sensors_21: MobileSensorNode {
        @display("i=misc/sensor;p=4335.26,7244.24");
    }

    sensors_22: MobileSensorNode {
        @display("i=misc/sensor;p=540.93,6768.23");
    }

    sensors_23: MobileSensorNode {
        @display("i=misc/sensor;p=940.74,6918.34");
    }

    sensors_24: MobileSensorNode {
        @display("i=misc/sensor;p=1290.72,6944.38");
    }

    sensors_25: MobileSensorNode {
        @display("i=misc/sensor;p=3557.33,6792.92");
    }

    sensors_26: MobileSensorNode {
        @display("i=misc/sensor;p=3892.89,6966.33");
    }

    sensors_27: MobileSensorNode {
        @display("i=misc/sensor;p=5104.74,6989.27");
    }

    sensors_28: MobileSensorNode {
        @display("i=misc/sensor;p=5726.23,6818.33");
    }

    sensors_29: MobileSensorNode {
        @display("i=misc/sensor;p=1214.19,6563.15");
    }

    sensors_30: MobileSensorNode {
        @display("i=misc/sensor;p=2018.92,6638.03");
    }

    sensors_31: MobileSensorNode {
        @display("i=misc/sensor;p=2586.50,6639.01");
    }

    sensors_32: MobileSensorNode {
        @display("i=misc/sensor;p=2983.32,6668.72");
    }

    sensors_33: MobileSensorNode {
        @display("i=misc/sensor;p=4340.44,6623.16");
    }

    sensors_34: MobileSensorNode {
        @display("i=misc/sensor;p=4734.19,6722.47");
    }

    sensors_35: MobileSensorNode {
        @display("i=misc/sensor;p=5414.67,6560.85");
    }

    sensors_36: MobileSensorNode {
        @display("i=misc/sensor;p=161.47,6492.49");
    }

    sensors_37: MobileSensorNode {
        @display("i=misc/sensor;p=748.70,6455.60");
    }

    sensors_38: MobileSensorNode {
        @display("i=misc/sensor;p=1726.77,6412.20");
    }

    sensors_39: MobileSensorNode {
        @display("i=misc/sensor;p=4960.13,6322.72");
    }

    sensors_40: MobileSensorNode {
        @display("i=misc/sensor;p=451.41,6022.84");
    }

    sensors_41: MobileSensorNode {
        @display("i=misc/sensor;p=1377.15,6250.72");
    }

    sensors_42: MobileSensorNode {
        @display("i=misc/sensor;p=2262.59,6244.23");
    }

    sensors_43: MobileSensorNode {
        @display("i=misc/sensor;p=2639.72,6232.58");
    }

    sensors_44: MobileSensorNode {
        @display("i=misc/sensor;p=2998.47,6191.95");
    }

    sensors_45: MobileSensorNode {
        @display("i=misc/sensor;p=3658.37,6115.85");
    }

    sensors_46: MobileSensorNode {
        @display("i=misc/sensor;p=4339.22,6147.20");
    }

    sensors_47: MobileSensorNode {
        @display("i=misc/sensor;p=4682.90,6071.98");
    }

    sensors_48: MobileSensorNode {
        @display("i=misc/sensor;p=5294.87,6037.52");
    }

    sensors_49: MobileSensorNode {
        @display("i=misc/sensor;p=5859.25,6245.03");
    }

    sensors_50: MobileSensorNode {
        @display("i=misc/sensor;p=1028.10,5858.00");
    }

}

